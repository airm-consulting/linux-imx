#include <linux/bits.h>
#include <linux/serial.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <linux/delay.h>

//#define DEBUG_PUL

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,14,0)
//this only come in a more recent kernel code
//https://github.com/torvalds/linux/blob/master/include/linux/bitops.h
#include <linux/bitops.h>
#define calculate_even_parity(n) parity8(n)
#define calculate_odd_parity(n) !parity8(n)
#endif
#define BITUART_MAX_BAUD 9600
#define UART_START_DELAY_NS 25000L
#define UART_STOP_DELAY_NS 25000L
#define OVERSAMPLE_RATE 10
#define DRIVER_NAME "bit-uart"
#define DEV_NAME "ttyBITS"
#define MAX_BITUART_PORT 2
#define BITUART_OVRE 1
#define PORT_GENERIC (-1)
#define SDI12BRK_DELAY_MS 12L
#define BSDI12BRK _IO('B', 1)

typedef enum {
    NO_PARITY,
    ODD_PARITY,
    EVEN_PARITY,
    MARK_PARITY,
    SPACE_PARITY
} _PARITY_OPTION_t;

typedef enum {
    TX_IDLE,
    TX_START_BIT,
    TX_DATA_BIT,
    TX_PARITY_BIT,
    TX_STOP_BIT
} _TX_STATE_t;

typedef enum {
    RX_IDLE,
    RX_WAIT_START_BIT,
    RX_DATA_BIT,
    RX_PARITY_BIT,
    RX_STOP_BIT
} _RX_STATE_t;

struct bituart_port {
    struct uart_port port;

    struct gpio_desc *gpio_tx;
    struct gpio_desc *gpio_rx;

    struct gpio_desc *gpio_rts;
    
    struct hrtimer timer_tx;
    struct hrtimer timer_rx;
    struct hrtimer timer_stop_tx;
    
    unsigned int inverted_tx:1;
    unsigned int inverted_rx:1;
    unsigned int half_duplex:1;
    unsigned int single_wire:1;
    unsigned int rx_enable:1;
    unsigned int tx_enable:1;
    //unsigned int has_dir_ctrl:1;
    unsigned int dir_ctrl_invert:1;
    unsigned int parse_parity_frame:1;//incorrect detection of stop and parity
    unsigned int ignore_parity_frame:1;//
    unsigned int parse_break:1;//a continuous 0 on rx-line
    //unsigned int parse_overrun:1;//new data is lost

    unsigned int data_bit_no;
    unsigned int stop_bit_no;
    _PARITY_OPTION_t parity;//0 = no parity, 1 = odd parity, 2 = even parity

    _TX_STATE_t tx_state;
    _RX_STATE_t rx_state;

    ktime_t period;
    ktime_t period_sample;

    //rx-runtime var
    int rx_bit_index;
    unsigned int rx_character;
    int start_bit_counter;
    int rx_parity;
    unsigned int parity_correct:1;

    //tx-rutime var
    int tx_bit_index;
    unsigned char tx_character;
    int tx_c_parity;

#ifdef DEBUG_PUL
    struct gpio_desc *gpio_dbg;
    unsigned int has_gpio_dbg:1;
#endif
};
static DECLARE_BITMAP(bituart_port_id_in_use, MAX_BITUART_PORT);

static inline struct bituart_port * to_bituart_port(struct uart_port * port)
{
    return container_of(port, struct bituart_port, port);
}

static void bituart_set_termios(struct uart_port *port, struct ktermios *termios,
                        const struct ktermios *oldtermios)
{
    struct bituart_port * bport = to_bituart_port(port);
    unsigned int baud; 
    unsigned long flags;
    unsigned int size = (termios->c_cflag & CSIZE);
    
    spin_lock_irqsave(&port->lock, flags);

    switch (size)
    {
        case CS5:
            bport->data_bit_no = 5;
            break;
        case CS6:
            bport->data_bit_no = 6;
            break;
        case CS7:
            bport->data_bit_no = 7;
            break;
        default:
            bport->data_bit_no = 8;
            break;
    }
    pr_debug("data sz:%d\n", bport->data_bit_no);

    baud = uart_get_baud_rate(port, termios, oldtermios, 0, BITUART_MAX_BAUD);
    bport->period = ktime_set(0, 1000000000 / baud);
    bport->period_sample = ktime_set(0, 1000000000 / baud / OVERSAMPLE_RATE);
    pr_debug("baud:%d\n", baud);
    
    if (termios->c_cflag & CSTOPB)
        bport->stop_bit_no = 2;
    else
        bport->stop_bit_no = 1;
    
    if (termios->c_cflag & PARENB)
    {
        if (termios->c_cflag & CMSPAR)
        {
            if (termios->c_cflag & PARODD)
            {
                bport->parity = MARK_PARITY;
            }
            else
            {
                bport->parity = SPACE_PARITY;
            }
        }
        else if (termios->c_cflag & PARODD)
        {
            bport->parity = ODD_PARITY;
            pr_debug("odd parity\n");
        }
        else
        {
            bport->parity = EVEN_PARITY;
            pr_debug("even parity\n");
        }
    }
    else
    {
        bport->parity = NO_PARITY;
        pr_debug("no parity\n");
    }

    if (!(termios->c_cflag & CREAD))
    {
        bport->rx_enable = 0;
    }
    if (termios->c_iflag & INPCK)
    {
        bport->parse_parity_frame = 1;
    }
    else
    {
        bport->parse_parity_frame = 0;
    }
    if (termios->c_iflag & (BRKINT | PARMRK))
    {
        bport->parse_break = 1;
    }
    else
    {
        bport->parse_break = 0;
    }
    //bport->parse_overrun=1;
    //port->ignore_status_mask = 0;
    if (termios->c_iflag & IGNPAR)
    {
        bport->ignore_parity_frame=1;
    }
    else
    {
        bport->ignore_parity_frame=0;
    }
    if (termios->c_iflag & IGNBRK)
    {
        bport->parse_break=0;
        if (termios->c_iflag & IGNPAR)
        {
            //bport->parse_overrun=0;
            //port->ignore_status_mask |= BITUART_OVRE;
            //we don't detect overrun at all
        }
    }

    if ((bport->port.rs485.flags & SER_RS485_ENABLED))
    {
        if (bport->port.rs485.flags & SER_RS485_RX_DURING_TX)
        {
            if (bport->single_wire)
            {
                pr_err("not possible for single line to do full duplex\n");
            }
        }
        else
        {
            bport->half_duplex = 1;   
        }
    }

    spin_unlock_irqrestore(&port->lock, flags);
}

static int bituart_rs485_config(struct uart_port * port, struct ktermios *termios, 
                                struct serial_rs485 * rs485conf)
{
    int ret=0;
    struct bituart_port * bport = to_bituart_port(port);
    if (rs485conf->flags & SER_RS485_ENABLED)
    {
        if (!(rs485conf->flags & SER_RS485_RX_DURING_TX))
        {
            bport->half_duplex = 1;
        }
        else
        {
            if (bport->single_wire)
            {
                pr_err("cannot do full duplex with single wire\n");
                ret = -EINVAL;
            }
        }
    }
    return ret;
}

static void bituart_stop_rx_sync(struct uart_port * port)
{
    struct bituart_port * bport = to_bituart_port(port);
    bport->rx_state = RX_IDLE;
    hrtimer_cancel(&bport->timer_rx);
}

static void bituart_stop_rx(struct uart_port * port)
{
    struct bituart_port * bport = to_bituart_port(port);
    bport->rx_state = RX_IDLE;
}

static void bituart_start_rx(struct uart_port * port)
{
    struct bituart_port * bport = to_bituart_port(port);
    if (!bport->rx_enable) return;
    if (bport->half_duplex)
    {
        gpiod_direction_input(bport->gpio_rx);
    }
#ifdef DEBUG_PUL
    if (bport->has_gpio_dbg)
        gpiod_direction_output(bport->gpio_dbg, 0);
#endif
    bport->rx_state = RX_WAIT_START_BIT;
    bport->rx_bit_index = -1;
    bport->rx_character = 0;
    bport->start_bit_counter = 0;
    bport->parity_correct = 0;
    hrtimer_start(&bport->timer_rx, bport->period_sample, HRTIMER_MODE_REL);
}

static void bituart_set_rts(struct uart_port * port, int send)
{
    struct bituart_port * bport = to_bituart_port(port);
    if (send)
    {
        //before send
        if ((port->rs485.flags & SER_RS485_ENABLED))
        {
            if ((port->rs485.flags & SER_RS485_RTS_ON_SEND))
            {
                if (!bport->dir_ctrl_invert)
                    gpiod_set_value(bport->gpio_rts, 1);
                else
                    gpiod_set_value(bport->gpio_rts, 0);
            }
            else if ((port->rs485.flags & SER_RS485_RTS_AFTER_SEND))
            {
                if (!bport->dir_ctrl_invert)
                    gpiod_set_value(bport->gpio_rts, 0);
                else
                    gpiod_set_value(bport->gpio_rts, 1);
            }
        }
    }
    else
    {
        //after send
        if ((port->rs485.flags & SER_RS485_ENABLED))
        {
            if ((port->rs485.flags & SER_RS485_RTS_AFTER_SEND))
            {
                if (!bport->dir_ctrl_invert)
                    gpiod_set_value(bport->gpio_rts, 1);
                else
                    gpiod_set_value(bport->gpio_rts, 0);
            }
            else if ((port->rs485.flags & SER_RS485_RTS_ON_SEND))
            {
                if (!bport->dir_ctrl_invert)
                    gpiod_set_value(bport->gpio_rts, 0);
                else
                    gpiod_set_value(bport->gpio_rts, 1);
            }
        } 
    }
} 

static void bituart_stop_tx_sync(struct uart_port* port)
{
    struct bituart_port * bport = to_bituart_port(port);
    bport->tx_state = TX_IDLE;
    bituart_set_rts(port, 0);//set rts after send
    hrtimer_cancel(&bport->timer_tx);
}

static void bituart_stop_tx(struct uart_port* port)
{
    struct bituart_port * bport = to_bituart_port(port);
    bport->tx_state = TX_IDLE;
    //the timer hardirq will stop itself so no need to stop it
    if ((port->rs485.flags & SER_RS485_ENABLED))
    {
        if ((port->rs485.flags & SER_RS485_RTS_AFTER_SEND))
        {
            if (port->rs485.delay_rts_after_send > 0)
            {
                hrtimer_start(&bport->timer_stop_tx, ktime_set(0, port->rs485.delay_rts_after_send * 1000000), HRTIMER_MODE_REL);
                return;
            }
            else
            {
                hrtimer_start(&bport->timer_stop_tx, ktime_set(0, UART_STOP_DELAY_NS), HRTIMER_MODE_REL);
                return;
            }
        }
        else if ((port->rs485.flags & SER_RS485_RTS_ON_SEND))
        {
            hrtimer_start(&bport->timer_stop_tx, ktime_set(0, UART_STOP_DELAY_NS), HRTIMER_MODE_REL);
            return;
        }
    } 
    if (bport->half_duplex)
    {
        bituart_start_rx(port);
    }
}

static enum hrtimer_restart stop_tx_callback(struct hrtimer *timer)
{
    struct bituart_port * bport = container_of(timer, struct bituart_port, timer_stop_tx);
    //deassert rts if needed
    bituart_set_rts(&bport->port, 0);
    if (bport->half_duplex)
    {
        bituart_start_rx(&bport->port);
    }
    return HRTIMER_NORESTART;
}

static void bituart_start_tx(struct uart_port* port)
{
    struct bituart_port * bport = to_bituart_port(port);
    if (!bport->tx_enable) return;
    pr_debug("starting tx from tty\n");
    if (bport->half_duplex)
    {
        bituart_stop_rx(port);
        gpiod_direction_output(bport->gpio_tx, (!bport->inverted_rx));
    }
    bituart_set_rts(port, 1);//set rts before sending
    bport->tx_state = TX_START_BIT;
    // Starts the TX timer if it is not already running.
    if (!hrtimer_active(&bport->timer_tx))
    {   
        if ((port->rs485.flags & SER_RS485_ENABLED) && 
            (port->rs485.flags & SER_RS485_RTS_ON_SEND) &&
            (port->rs485.delay_rts_before_send > 0))
        {
            hrtimer_start(&bport->timer_tx, ktime_set(0, port->rs485.delay_rts_before_send * 1000000), HRTIMER_MODE_REL);
        }
        else
        {
            hrtimer_start(&bport->timer_tx, ktime_set(0, UART_START_DELAY_NS), HRTIMER_MODE_REL);
        } 
    }
}

static int bituart_open(struct uart_port *port)
{
    struct bituart_port * bport = to_bituart_port(port);
    pr_debug("open port\n");
    if (!bport->half_duplex)
    {
        pr_debug("setting tx\n");
        if (bport->tx_enable)
            gpiod_set_value(bport->gpio_tx, !bport->inverted_tx);
    }
    bituart_set_rts(port, 0);//set rts after send, in this case, initial state is like when we're not sending
    bituart_start_rx(port);
    return 0;
}

static void bituart_close(struct uart_port *port)
{
    pr_debug("close port\n");
    bituart_stop_rx_sync(port);
    bituart_stop_tx_sync(port);
}

static unsigned int bituart_tx_empty(struct uart_port * port)
{
    struct bituart_port * bport = to_bituart_port(port);
    unsigned int ret;
    ret = (bport->tx_state == TX_IDLE)? TIOCSER_TEMT : 0;
    return ret;
}

static void bituart_set_mctrl(struct uart_port * port, unsigned int mctrl)
{
}

static unsigned int bituart_get_mctrl(struct uart_port * port)
{
    unsigned int ret = 0;
    ret |= (TIOCM_CAR|TIOCM_CTS|TIOCM_DSR);
    return ret;
}

static void bituart_enable_ms(struct uart_port * port)
{
}

static void bituart_break_ctl(struct uart_port * port, int break_state)
{
    struct bituart_port * bport = to_bituart_port(port);
    pr_debug("brk:%d\n", break_state);
    if (!bport->tx_enable) return;
    if (break_state != 0)
    {
        if (bport->half_duplex)
        {
            bituart_stop_rx_sync(port);
            gpiod_direction_output(bport->gpio_tx, !bport->inverted_tx);
        }
        bituart_stop_tx_sync(port);
        bituart_set_rts(port, 1);//set rts before tx
        gpiod_set_value(bport->gpio_tx, !!bport->inverted_tx);//0 for non-invert, 1 for invert
    }
    else
    {
        gpiod_set_value(bport->gpio_tx, !bport->inverted_tx);//1 for non-invert, 0 for invert
        if (bport->half_duplex)
        {
            bituart_stop_tx(&bport->port);//this will take care of rts and start rx
        }
    }
}

static void bituart_flush_buffer(struct uart_port * port)
{
    pr_debug("flushing output buffer\n");
    bituart_stop_tx(port);
}

static const char *bituart_uart_type(struct uart_port *port)
{
	return (port->type == PORT_GENERIC) ? NULL : NULL;
}

static void bituart_config_port(struct uart_port *port, int flags)
{
    if (flags & UART_CONFIG_TYPE)
        port->type = PORT_GENERIC;
}

static int bituart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
    return -EINVAL;
}

static int bituart_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
    int ret;
    switch (cmd)
    {
        case BSDI12BRK:
            pr_debug("sending a SDI-12 break\n");
            bituart_break_ctl(port, 1);
            msleep_interruptible(SDI12BRK_DELAY_MS);
            bituart_break_ctl(port, 0);
            ret = 0;
            break;
        default:
            ret = -ENOIOCTLCMD;
            break;
    }
    return ret;
}

//-----------------------------------------------------------------------------
// Internals
//-----------------------------------------------------------------------------

static const struct uart_ops bituart_uart_ops = 
{
    .tx_empty = bituart_tx_empty,
    .set_mctrl = bituart_set_mctrl,
    .get_mctrl = bituart_get_mctrl,
    .stop_tx = bituart_stop_tx,
    .start_tx = bituart_start_tx,
    .stop_rx = bituart_stop_rx,
    .enable_ms = bituart_enable_ms,
    .break_ctl	= bituart_break_ctl,
	.startup	= bituart_open,
	.shutdown	= bituart_close,
	.flush_buffer	= bituart_flush_buffer,
	.set_termios	= bituart_set_termios,
	.type		= bituart_uart_type,
	.config_port	= bituart_config_port,
	.verify_port	= bituart_verify_port,
    .ioctl       = bituart_ioctl,
};

#ifndef calculate_even_parity
int calculate_even_parity(unsigned char val)
{
    val ^= val >> 4;
	return (0x6996 >> (val & 0xf)) & 1;
}
#endif
#ifndef calculate_odd_parity
int calculate_odd_parity(unsigned char val)
{
    return (!calculate_even_parity(val));
}
#endif

static int dequeue_character(struct uart_port * port, unsigned char * c)
{
    struct bituart_port * bport = to_bituart_port(port);
    struct circ_buf *xmit = &port->state->xmit;
    int ret;
    spin_lock(&port->lock);
    if (port->x_char)
    { 
        (*c) = (port->x_char & GENMASK(bport->data_bit_no-1,0));
        ret = 1;
        port->icount.tx++;
        port->x_char = 0;
    }
    else
    {
        if (uart_circ_empty(xmit) || uart_tx_stopped(port))
        {
            ret = 0;
        }
        else
        {
            (*c) =  (xmit->buf[xmit->tail] & GENMASK(bport->data_bit_no-1,0));
            uart_xmit_advance(port, 1);
            ret = 1;
        }
    }
    spin_unlock(&port->lock);
    return ret;
}

static int get_queue_size(struct uart_port * port)
{
    struct circ_buf *xmit = &port->state->xmit;
    int pending;
    spin_lock(&port->lock);
    pending = uart_circ_chars_pending(xmit);
    if (pending < WAKEUP_CHARS)
        uart_write_wakeup(port);
    spin_unlock(&port->lock);
    return pending;
}

static enum hrtimer_restart handle_tx(struct hrtimer *timer)
{
    struct bituart_port * bport = container_of(timer, struct bituart_port, timer_tx);
    ktime_t current_time = ktime_get();
    enum hrtimer_restart result = HRTIMER_NORESTART;
    bool must_restart_timer = false;

    // Start bit.
    if (bport->tx_state == TX_START_BIT)
    {
        if (dequeue_character(&bport->port, &bport->tx_character))
        {
            if (bport->parity == ODD_PARITY)
            {
                bport->tx_c_parity = calculate_odd_parity(bport->tx_character);
            }
            else if (bport->parity == EVEN_PARITY) 
            {
                bport->tx_c_parity = calculate_even_parity(bport->tx_character);
            }
            else if (bport->parity == MARK_PARITY)
            {
                bport->tx_c_parity = 1;
            }
            else if (bport->parity == SPACE_PARITY)
            {
                bport->tx_c_parity = 0;
            }
            gpiod_set_value(bport->gpio_tx, (!!bport->inverted_tx));
            bport->tx_bit_index=0;
            must_restart_timer = true;
            bport->tx_state = TX_DATA_BIT;
        }
        else
        {
            if (bport->half_duplex)
            {
                bituart_stop_tx(&bport->port);
            }
            bport->tx_state = TX_IDLE;
        }
    }
    // Data bits.
    else if (bport->tx_state == TX_DATA_BIT)
    {
        gpiod_set_value(bport->gpio_tx, !!(bport->inverted_tx ^ (1 & (bport->tx_character >> bport->tx_bit_index))));
        if (bport->tx_bit_index == (bport->data_bit_no - 1))
        {
            if (bport->parity == ODD_PARITY || bport->parity == EVEN_PARITY 
                || bport->parity == MARK_PARITY || bport->parity == SPACE_PARITY)
            {
                bport->tx_bit_index++;
                must_restart_timer = true;
                bport->tx_state = TX_PARITY_BIT;
            }
            else if (bport->stop_bit_no == 0)
            {
                bport->tx_bit_index = -1;
                must_restart_timer = get_queue_size(&bport->port) > 0;
                if (must_restart_timer)
                {
                    bport->tx_state = TX_START_BIT;
                }
                else
                {
                    if (bport->half_duplex)
                    {
                        bituart_stop_tx(&bport->port);
                    }
                    bport->tx_state = TX_IDLE;
                }
            }
            else
            {
                bport->tx_bit_index++;
                must_restart_timer = true;
                bport->tx_state = TX_STOP_BIT;
            }
        }
        else
        {
            bport->tx_bit_index++;
            must_restart_timer = true;
        }
    }
    // Parity bit.
    else if (bport->tx_state == TX_PARITY_BIT)
    {
        gpiod_set_value(bport->gpio_tx, !!(bport->inverted_tx ^ (bport->tx_c_parity)));
        if (bport->stop_bit_no == 0)
        {
            bport->tx_bit_index = -1;
            must_restart_timer = get_queue_size(&bport->port) > 0;
            if (must_restart_timer)
            {
                bport->tx_state = TX_START_BIT;
            }
            else
            {
                if (bport->half_duplex)
                {
                    bituart_stop_tx(&bport->port);
                }
                bport->tx_state = TX_IDLE;
            }
        }
        else
        {
            must_restart_timer = true;
            bport->tx_state = TX_STOP_BIT;
        }
    }
    // Stop bit.
    else if (bport->tx_state == TX_STOP_BIT)
    {
        gpiod_set_value(bport->gpio_tx, !bport->inverted_tx);
        bport->tx_character = 0;
        if ((bport->tx_bit_index - bport->data_bit_no) == (bport->stop_bit_no - 1))
        {
            bport->tx_bit_index = -1;
            must_restart_timer = get_queue_size(&bport->port) > 0;
            if (must_restart_timer)
            {
                bport->tx_state = TX_START_BIT;
            }
            else
            {
                if (bport->half_duplex)
                {
                    bituart_stop_tx(&bport->port);
                }
                bport->tx_state = TX_IDLE;
            }
        }
        else
        {
            bport->tx_bit_index++;
            must_restart_timer = true;
        }
    }

    // Restarts the TX timer.
    if (must_restart_timer)
    {
        hrtimer_forward(timer, current_time, bport->period);
        result = HRTIMER_RESTART;
    }

    return result;
}

static void receive_character(struct uart_port * port, unsigned char c, unsigned char flg)
{
    spin_lock(&port->lock);
    port->icount.rx++;
    if (flg == TTY_PARITY)
    {
        port->icount.parity++;
    }
    else if (flg == TTY_FRAME)
    {
        port->icount.frame++;
    }
    else if (flg == TTY_BREAK)
    {
        port->icount.brk++;
        if (uart_handle_break(port)) goto __end_receive;
    }
    if (uart_handle_sysrq_char(port, c))
    {
        return;
    }
    uart_insert_char(port, 0, BITUART_OVRE, c, flg);
    tty_flip_buffer_push(&port->state->port);
__end_receive:
    spin_unlock(&port->lock);
}

static enum hrtimer_restart handle_rx(struct hrtimer *timer)
{
    struct bituart_port * bport = container_of(timer, struct bituart_port, timer_rx);
    ktime_t current_time = ktime_get();
    int c_parity = 0;
    enum hrtimer_restart result = HRTIMER_NORESTART;
    bool must_restart_timer = false;
    int bit_value;
    if (bport->rx_state == RX_IDLE)
    {
        return result;
    }
    bit_value = bport->inverted_rx ^ (!!gpiod_get_value(bport->gpio_rx));
#ifdef DEBUG_PUL
    //gpiod_set_value(gpio_dbg, !gpiod_get_value(gpio_dbg));
    //gpiod_set_value(gpio_dbg, 0);
#endif
    // if (bport->rx_state != 1)
    //     pr_debug("%d", bport->rx_state);
    // Start bit.
    if (bport->rx_state == RX_WAIT_START_BIT)
    {
        if (bit_value == 1)
        {
            bport->start_bit_counter = 0;
        }
        else if (++bport->start_bit_counter == (OVERSAMPLE_RATE/2))
        {
            bport->rx_bit_index=0; 
            bport->rx_character = 0;
            bport->rx_state = RX_DATA_BIT;
#ifdef DEBUG_PUL
            if (bport->has_gpio_dbg)
                gpiod_set_value(bport->gpio_dbg, 1);
#endif            
        }
        must_restart_timer = true;
    }
    // Data bits.
    else if (bport->rx_state == RX_DATA_BIT)
    {
#ifdef DEBUG_PUL
        if (bport->has_gpio_dbg)
            gpiod_set_value(bport->gpio_dbg, !gpiod_get_value(bport->gpio_dbg));
    //gpiod_set_value(gpio_dbg, 0);
#endif
        if (bit_value == 0)
        {
            bport->rx_character &= (~(1 << bport->data_bit_no));
        }
        else
        {
            bport->rx_character |= (1 << bport->data_bit_no);
        }
        bport->rx_character >>= 1;
        if (bport->rx_bit_index == (bport->data_bit_no - 1))
        {
            if (bport->parity == ODD_PARITY || bport->parity == EVEN_PARITY
                || bport->parity == MARK_PARITY || bport->parity == SPACE_PARITY)
            {
                bport->rx_bit_index++;
                must_restart_timer = true;
                bport->rx_state = RX_PARITY_BIT;
            }
            else if (bport->stop_bit_no == 0)
            {
                //we might not be able to detect break or just 0x00 without stop bit
                receive_character(&bport->port, bport->rx_character, TTY_NORMAL);
                bport->rx_bit_index = -1;
                bport->start_bit_counter = 0;
                //since we don't have stop bit, we can only assume next bit is start bit
                bport->rx_state = RX_WAIT_START_BIT;                
                must_restart_timer = true;
            }
				else
				{
					bport->rx_bit_index++;
					must_restart_timer = true;
					bport->rx_state = RX_STOP_BIT;
				}
        }
        else
        {
            bport->rx_bit_index++;
            must_restart_timer = true;
        }
    }
    // Parity bit.
    else if (bport->rx_state == RX_PARITY_BIT)
    {
#ifdef DEBUG_PUL
        if (bport->has_gpio_dbg)
            gpiod_set_value(bport->gpio_dbg, !gpiod_get_value(bport->gpio_dbg));
    //gpiod_set_value(gpio_dbg, 0);
#endif
        if (bport->parity == ODD_PARITY)
        {
            c_parity = calculate_odd_parity(bport->rx_character);
        }
        else if (bport->parity == EVEN_PARITY)
        {
            c_parity = calculate_even_parity(bport->rx_character);
        }
        else if (bport->parity == MARK_PARITY)
        {
            c_parity = 1;
        }
        else if (bport->parity == SPACE_PARITY)
        {
            c_parity = 0;
        }
        bport->rx_parity = bit_value;
        if (bit_value != c_parity)
        {
            // wrong parity bit, possible missampling
            bport->parity_correct=0;
        }
        else
        {
            bport->parity_correct=1;
        }
        if (bport->stop_bit_no == 0)
        {
            if (bport->rx_character==0 && bit_value == 0)
            {
                //break detected
                if (bport->parse_break)
                {
                    // parse TTYBREAK
                    receive_character(&bport->port, bport->rx_character, TTY_BREAK);
                }
            }
            else
            {
                if (!bport->parse_parity_frame)
                {
                    receive_character(&bport->port, bport->rx_character, TTY_NORMAL);
                }
                else
                {
                    if (bport->parity_correct)
                    {
                        receive_character(&bport->port, bport->rx_character, TTY_NORMAL);
                    }
                    else
                    {
                        if (!bport->ignore_parity_frame)
                        {
                            // parse TTY_PARITY
                            receive_character(&bport->port, bport->rx_character, TTY_PARITY);
                        }
                    }
                }
            }
            bport->rx_bit_index = -1;
            bport->start_bit_counter = 0;
            //since we don't have stop bit, we can only assume next bit is start bit
            bport->rx_state = RX_WAIT_START_BIT;                
            must_restart_timer = true;
        }
        else
        {
            must_restart_timer = true;
            bport->rx_state = RX_STOP_BIT;
        }
    }
    // Stop bit.
    else if (bport->rx_state == RX_STOP_BIT)
    {
#ifdef DEBUG_PUL
        //gpiod_set_value(gpio_dbg, !gpiod_get_value(gpio_dbg));
        if (bport->has_gpio_dbg)
            gpiod_set_value(bport->gpio_dbg, 0);
#endif            
        if (bit_value == 0)
        {
            if (bport->rx_character == 0 && bport->rx_parity == 0)
            {
                //break detected
                if (bport->parse_break)
                {
                    // parse TTYBREAK
                    receive_character(&bport->port, bport->rx_character, TTY_BREAK);
                }
            }
            else
            {
                // wrong stop bit condition, possible missampling
                // framing error
                if (!bport->parse_parity_frame)
                {
                    receive_character(&bport->port, bport->rx_character, TTY_NORMAL);
                }
                else
                {
                    if (!bport->ignore_parity_frame)
                    {
                        // parse FRAMING ERR
                        receive_character(&bport->port, bport->rx_character, TTY_FRAME);
                    }
                }
            } 
            bport->rx_bit_index = -1;
            bport->start_bit_counter = 0;
            bport->rx_state = RX_WAIT_START_BIT;
            must_restart_timer = true;
        }
        else if ((bport->rx_bit_index - bport->data_bit_no) == (bport->stop_bit_no - 1))
        {
            if (!bport->parse_parity_frame)
            {
                receive_character(&bport->port, bport->rx_character, TTY_NORMAL);
            }
            else
            {
                if (bport->parity_correct)
                {
                    receive_character(&bport->port, bport->rx_character, TTY_NORMAL);
                }
                else
                {
                    if (!bport->ignore_parity_frame)
                    {
                        // parse TTY_PARITY
                        receive_character(&bport->port, bport->rx_character, TTY_PARITY);
                    }
                }
            }
            bport->rx_bit_index = -1;
            bport->start_bit_counter = 0;
            bport->rx_state = RX_WAIT_START_BIT;
            must_restart_timer = true;
        }
        else
        {
            bport->rx_bit_index++;
            must_restart_timer = true;
        }
    }

    // Restarts the RX timer.
    if (must_restart_timer)
    {
        if (bport->rx_state == RX_WAIT_START_BIT)
        {
            hrtimer_forward(timer, current_time, bport->period_sample);
        }
        else
        {
            hrtimer_forward(timer, current_time, bport->period);
        }
        result = HRTIMER_RESTART;
    }
    return result;
}

static const struct serial_rs485 bituart_rs485_supported_half_duplex = {
    .flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND | SER_RS485_RTS_AFTER_SEND,
    .delay_rts_before_send = 1, //delay is necessary to ensure diff line stablilize
    .delay_rts_after_send = 1,
};

static const struct serial_rs485 bituart_rs485_supported_full_duplex = {
    .flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND | SER_RS485_RTS_AFTER_SEND |
            SER_RS485_RX_DURING_TX,
    .delay_rts_before_send = 1, //delay is necessary to ensure diff line stablilize
    .delay_rts_after_send = 1,
};

static struct uart_driver bituart_uart_driver = {
    .owner = THIS_MODULE,
    .driver_name = DRIVER_NAME,
    .dev_name = DEV_NAME,
    .major = 0,
    .minor = 0,
    .nr = MAX_BITUART_PORT,
    //no console support yet
};

static int bituart_probe(struct platform_device *pdev)
{
    int ret = 0;
    struct device *dev = &pdev->dev;
    struct bituart_port *bport;
    bport = devm_kzalloc(dev, sizeof(*bport), GFP_KERNEL);
    if (!bport)
        return -ENOMEM;
    //allocate serial port id
    ret = of_alias_get_id(pdev->dev.of_node, "serial");
    if (ret < 0)
    {
        //we'll allocate one id
        ret = find_first_zero_bit(bituart_port_id_in_use, MAX_BITUART_PORT);
    }
    if (ret >= MAX_BITUART_PORT)
    {
        return dev_err_probe(dev, -ENODEV, "%p all bituart port is enumerated\n", dev_fwnode(dev));
    }
    if (test_and_set_bit(ret, bituart_port_id_in_use))
    {
        return dev_err_probe(dev, -EBUSY, "%p bituart port is already in used\n", dev_fwnode(dev));
    }
    bport->port.line = ret;

    bport->single_wire = device_property_read_bool(dev, "bituart,single-wire");
    bport->gpio_rx = devm_gpiod_get(dev, "rx", GPIOD_ASIS);
    bport->gpio_tx = devm_gpiod_get(dev, "tx", GPIOD_ASIS);
    if (IS_ERR(bport->gpio_rx) && IS_ERR(bport->gpio_tx))
    {
        return dev_err_probe(dev, PTR_ERR(bport->gpio_rx),
                    "%pfw, could not get any rx or tx gpio\n",
                    dev_fwnode(dev));
    }
    if (IS_ERR(bport->gpio_rx))
    {
        if (!bport->single_wire)
        {
            pr_debug("rx disabled\n");
            bport->rx_enable = 0;
        }
    }
    else
    {
        bport->rx_enable = 1;
        if (gpiod_cansleep(bport->gpio_rx))
        {
            return dev_err_probe(dev, -EINVAL,
                "%pfw, sleeping rx GPIO not supported\n",
                dev_fwnode(dev));
        }
    }
    if (IS_ERR(bport->gpio_tx))
    {
        if (!bport->single_wire)
        {
            pr_debug("tx disabled\n");
            bport->tx_enable = 0;
        }
    }
    else
    {
        bport->tx_enable = 1;
        if (gpiod_cansleep(bport->gpio_tx))
        {
            return dev_err_probe(dev, -EINVAL,
                "%pfw, sleeping tx GPIO not supported\n",
                dev_fwnode(dev));
        }
    }
    if (bport->single_wire)
    {
        pr_debug("single wire mode\n");
        //with single txrx line, only half-duplex is possible
        bport->half_duplex = 1;
        bport->tx_enable = 1;
        bport->rx_enable = 1;
        if (IS_ERR(bport->gpio_rx))
            bport->gpio_rx = bport->gpio_tx;
        if (IS_ERR(bport->gpio_tx))
            bport->gpio_tx = bport->gpio_rx;
    }

#ifdef DEBUG_PUL
    bport->gpio_dbg = devm_gpiod_get_optional(dev, "dbg", GPIOD_ASIS | GPIOD_FLAGS_BIT_NONEXCLUSIVE);
    if (!IS_ERR(bport->gpio_dbg))
    {
        pr_debug("has debug gpio pin\n");
        bport->has_gpio_dbg = 1;
    }
#endif

    bport->inverted_tx = device_property_read_bool(dev, "bituart,inverted-tx");
    bport->inverted_rx = device_property_read_bool(dev, "bituart,inverted-rx");
    bport->dir_ctrl_invert = device_property_read_bool(dev, "rs485-rts-active-low");

    bport->port.dev = dev;
    bport->port.type = PORT_GENERIC;
    bport->port.fifosize = 1;
    bport->port.has_sysrq = 0;//we do not support console now
    bport->port.ops = &bituart_uart_ops;
    bport->port.rs485_config = bituart_rs485_config;
    if (device_property_present(dev, "rts-gpios"))
    {
        pr_debug("rts gpio detected\n");
        if (bport->single_wire || !bport->rx_enable || !bport->tx_enable)
            bport->port.rs485_supported = bituart_rs485_supported_half_duplex;
        else
            bport->port.rs485_supported = bituart_rs485_supported_full_duplex;

    }
    //this is a hacky way to handle a shared gpio pin, but i cannot find any better solution
    //we are not using the mctrl gpio init function as our modem gpio is shared
    bport->gpio_rts = devm_gpiod_get_optional(dev, "rts", GPIOD_OUT_LOW | GPIOD_FLAGS_BIT_NONEXCLUSIVE);
    if (IS_ERR(bport->gpio_rts))
    {
        return dev_err_probe(dev, PTR_ERR(bport->gpio_rts),
            "%pfw, could not get mctrl\n",
            dev_fwnode(dev));
    }
    ret = uart_get_rs485_mode(&bport->port);
    if (ret)
    {
        return ret;
    }

    if (((bport->tx_enable && !bport->rx_enable)||(bport->rx_enable && !bport->tx_enable)||bport->single_wire) && 
        (bport->port.rs485.flags & SER_RS485_ENABLED) &&
        (bport->port.rs485.flags & SER_RS485_RX_DURING_TX))
    {
        //it is physically not possible to do rx during tx when there's only 1 txrx line
        return dev_err_probe(dev, -EINVAL,
                "%pfw, cannot do rx during tx when only single tx rx line is available\n",
                dev_fwnode(dev));
    }
    bport->port.flags = UPF_BOOT_AUTOCONF;

    hrtimer_init(&bport->timer_tx, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    bport->timer_tx.function = handle_tx;
    hrtimer_init(&bport->timer_rx, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    bport->timer_rx.function = handle_rx;

    hrtimer_init(&bport->timer_stop_tx, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    bport->timer_stop_tx.function = stop_tx_callback;

    if (bport->tx_enable && !bport->single_wire)
    {
        //only initialize tx when not in half-duplex mode
        if ((ret=gpiod_direction_output(bport->gpio_tx, (!bport->inverted_tx))) != 0)
        {
            return dev_err_probe(dev, ret, "err setting tx gpio\n");
        }
    }
    if (bport->rx_enable)
    {
        if ((ret=gpiod_direction_input(bport->gpio_rx)) != 0)
        {
            dev_err(dev, "err setting rx gpio\n");
            goto err_port;
        }
    }
    
    platform_set_drvdata(pdev, bport);
    
    ret = uart_add_one_port(&bituart_uart_driver, &bport->port);
    if (ret)
        goto err_port;

    pr_info("%s: %s probed OK\n", KBUILD_MODNAME, dev_name(dev));
    return 0;

err_port:
    if (!bport->single_wire && bport->tx_enable)
    {
        gpiod_direction_input(bport->gpio_tx);
    }
    return ret;
}

static int bituart_remove(struct platform_device *pdev)
{
    int ret = 0;
    struct bituart_port *bport = platform_get_drvdata(pdev);

    ret = uart_remove_one_port(&bituart_uart_driver, &bport->port);

    if (!bport->single_wire && bport->tx_enable)
    {
        gpiod_direction_input(bport->gpio_tx);
    }

    clear_bit(bport->port.line, bituart_port_id_in_use);

    return ret;
}

static int __maybe_unused bituart_suspend(struct device *dev)
{
    struct bituart_port * bport = dev_get_drvdata(dev);
    uart_suspend_port(&bituart_uart_driver, &bport->port);
    return 0;
}

static int __maybe_unused bituart_resume(struct device * dev)
{
    struct bituart_port * bport = dev_get_drvdata(dev);
    uart_resume_port(&bituart_uart_driver, &bport->port);
    return 0;
}

static SIMPLE_DEV_PM_OPS(bituart_pm_ops, bituart_suspend,
			 bituart_resume);

static const struct of_device_id bituart_match[] = {
    { .compatible = "bit-uart" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bituart_match);

static struct platform_driver bituart_platform_driver = {
    .probe = bituart_probe,
    .remove = bituart_remove,
    .driver = {
        .name = DRIVER_NAME,
        .pm = &bituart_pm_ops,
        .of_match_table = of_match_ptr(bituart_match),
    },
};

static int __init bituart_init(void)
{
    int ret;
    ret = uart_register_driver(&bituart_uart_driver);
    if (ret)
        return ret;
    
    ret = platform_driver_register(&bituart_platform_driver);
    if (ret)
        uart_unregister_driver(&bituart_uart_driver);

    pr_info("%s: driver initialized\n", KBUILD_MODNAME);
    return ret;
}

static void __exit bituart_exit(void)
{
    platform_driver_unregister(&bituart_platform_driver);
    uart_unregister_driver(&bituart_uart_driver);
}

module_init(bituart_init);
module_exit(bituart_exit);

MODULE_DESCRIPTION("Bit banging serial port driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
