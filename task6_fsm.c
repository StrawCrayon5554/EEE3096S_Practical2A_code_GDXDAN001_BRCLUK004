/**
  ******************************************************************************
  * @file    task6_fsm.c
  * @brief   TASK 6 : NON-BLOCKING EEPROM TRANSACTION STATE MACHINE
  *
  * Restructure the Task 4 transaction so the main loop never stops:
  *
  *   request -> write-enable -> write -> wait for EEPROM
  *           -> read-back -> verify -> result
  *
  * Rules (handout, Task 6):
  *   - No HAL_Delay(), and no software busy-wait for the EEPROM's internal
  *     write cycle. You may use HAL_GetTick() to decide when the next status
  *     check is due.
  *   - Each call does a small amount of work, updates the state, and returns.
  *   - Do not put the whole transaction inside one blocking function called
  *     from the loop.
  *   - While a transaction is in progress the loop must still respond to PA3.
  *
  * Controls: PA0 starts, PA3 aborts. PB0..PB7 show the last byte read, PB11
  * (green) a successful verification, PB10 (red) a failed one.
  ******************************************************************************
  */

#include "prac2a.h"

/* TODO 6.1  Design your states. You need enough of them to tell apart at
 *           least: idle, write preparation, write transaction, EEPROM busy /
 *           status checking, read transaction, verification, and success or
 *           failure.
 *
 *           Draw the diagram first. It must show the initial state, the
 *           condition on every transition, and the success, failure and
 *           abort paths.
 *
 *           typedef enum { ... } ee_state_t;
 */

typedef enum {
    EE_STATE_IDLE = 0,
    EE_STATE_WRITE_ENABLE,
    EE_STATE_WRITE_DATA,
    EE_STATE_POLL_STATUS,
    EE_STATE_READ_BACK,
    EE_STATE_VERIFY,
    EE_STATE_SUCCESS,
    EE_STATE_FAILURE
} ee_state_t;



volatile uint8_t ee_state       = 0u;
volatile uint8_t ee_last_read   = 0u;
volatile uint8_t ee_use_fsm     = 1u;

#define EE_POLL_INTERVAL_MS  5u

static uint32_t ee_poll_start_ms = 0u;
static uint32_t ee_next_poll_ms  = 0u;

void update_eeprom_state_machine(uint32_t now)
{
    /* TODO 6.2  One step of your state machine.
     *
     *   - btn_start_edge (PA0) starts a transaction from idle.
     *   - btn_abort_edge (PA3) abandons the transaction in progress and returns
     *     to idle. Leave the SPI bus in a state the next transaction can use.
     *   - While the EEPROM is busy writing, do NOT wait in here. Work out when
     *     the next status check is due, remember it, and return.
     *   - Decide that the write has finished from the status register, never
     *     from elapsed time alone. Also decide what happens if the EEPROM never
     *     reports ready.
     *   - Store the byte read back in ee_last_read.
     *   - Clear each button edge once you have acted on it. */



	/* Abort overrides everything except idle itself. */
	if (btn_abort_edge && (ee_state != EE_STATE_IDLE))
	{
		btn_abort_edge = 0u;
		ee_state = EE_STATE_IDLE;
		return;
	}
	btn_abort_edge = 0u;

	switch (ee_state)
	{
	case EE_STATE_IDLE:
		if (btn_start_edge)
		{
			btn_start_edge = 0u;
			ee_state = EE_STATE_WRITE_ENABLE;
		}
		break;

	case EE_STATE_WRITE_ENABLE:
		eeprom_write_enable();
		ee_state = EE_STATE_WRITE_DATA;
		break;

	case EE_STATE_WRITE_DATA:
		eeprom_cs_low();
		(void)spi_transfer(EEPROM_CMD_WRITE);
		(void)spi_transfer((uint8_t)(eeprom_test_addr >> 8));
		(void)spi_transfer((uint8_t)(eeprom_test_addr & 0xFFu));
		(void)spi_transfer(eeprom_test_byte);
		eeprom_cs_high();

		ee_poll_start_ms = now;
		ee_next_poll_ms  = now;          /* check immediately on first pass */
		ee_state = EE_STATE_POLL_STATUS;
		break;

	case EE_STATE_POLL_STATUS:
		if ((int32_t)(now - ee_next_poll_ms) >= 0)
		{
			uint8_t sr = eeprom_read_status();

			if (!(sr & EEPROM_SR_RDY))
			{
				ee_state = EE_STATE_READ_BACK;
			}
			else if ((uint32_t)(now - ee_poll_start_ms) > EEPROM_WRITE_TIMEOUT_MS)
			{
				eeprom_timeout_count++;
				ee_state = EE_STATE_FAILURE;
			}
			else
			{
				ee_next_poll_ms = now + EE_POLL_INTERVAL_MS;
			}
		}
		break;

	case EE_STATE_READ_BACK:
		eeprom_read_value = eeprom_read_byte(eeprom_test_addr);
		ee_last_read = eeprom_read_value;
		ee_state = EE_STATE_VERIFY;
		break;

	case EE_STATE_VERIFY:
		eeprom_verify_ok = (eeprom_read_value == eeprom_test_byte) ? 1u : 0u;
		ee_state = eeprom_verify_ok ? EE_STATE_SUCCESS : EE_STATE_FAILURE;
		break;

	case EE_STATE_SUCCESS:
	case EE_STATE_FAILURE:
		ee_state = EE_STATE_IDLE;   /* one-shot result, then back to idle */
		break;

	default:
		ee_state = EE_STATE_IDLE;
		break;
	}
}


void update_outputs(void)
{
    /* TODO 6.3  PB0..PB7 show ee_last_read (leds_write_byte). Green after a
     *           successful verification, red after a failed one
     *           (status_leds_show). Decide what the status LEDs should show
     *           while a transaction is in progress, and after an abort. */
	leds_write_byte(ee_last_read);

	if ((ee_state == EE_STATE_IDLE) || (ee_state == EE_STATE_SUCCESS) || (ee_state == EE_STATE_FAILURE))
	{
		status_leds_show(eeprom_verify_ok ? STATUS_PASS : STATUS_FAIL);
	}
	/* else: transaction in progress -- leave status LEDs showing the
	   previous result, or turn both off here if you'd rather signal "busy"
	   explicitly. Check what status_leds_show() supports before adding a
	   third state. */
}

/* ==========================================================================
 * RUN_TASK 6 - given
 *
 * The main loop has exactly the shape the handout asks for:
 *     read_inputs();  update_eeprom_state_machine();  update_outputs();
 *
 * Set ee_use_fsm = 0 in Live Expressions to run the Task 4 blocking path on
 * PA0 instead - useful when you explain why yours is non-blocking. PC13 keeps
 * toggling as a heartbeat; watch it on the scope in both modes.
 * ========================================================================== */

void task6_setup(void)
{
    task1_gpio_init();
    eeprom_spi_init();
    board_io_init();

    eeprom_read_only_path();
    ee_last_read = eeprom_read_value;

    /* TODO 6.4  Your state machine starts in its idle state. Make sure the
     *           boot-time result (eeprom_verify_ok) still shows on the status
     *           LEDs, so a reset still shows green for the persistence test. */
    ee_state = EE_STATE_IDLE;
	update_outputs();   /* make sure boot-time result shows immediately */
}

void task6_loop(uint32_t now)
{
    task1_gpio_update(now);
    read_inputs(now);

    if (ee_use_fsm)
    {
        update_eeprom_state_machine(now);
        update_outputs();
    }
    else
    {
        if (btn_start_edge)
        {
            btn_start_edge = 0u;
            eeprom_write_verify_path();     /* Task 4: blocks for the write */
            ee_last_read = eeprom_read_value;
        }
        btn_abort_edge = 0u;
    }
}
