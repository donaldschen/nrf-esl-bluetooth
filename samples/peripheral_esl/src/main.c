/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic Electronic Shelf Label Service (ESL) application
 */

#include <stdio.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <host/id.h>
#include <zephyr/bluetooth/controller.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/gpio.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_power.h>
#if !NRF_POWER_HAS_RESETREAS
#include <hal/nrf_reset.h>
#endif
#include <zephyr/sys/poweroff.h>

#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>

#include "esl.h"
#include "esl_hw_impl.h"
#if defined(CONFIG_BT_ESL_VENDOR_SPECIFIC_SUPPORT)
#include "esl_vs_impl.h"
#endif /* CONFIG_BT_ESL_VENDOR_SPECIFIC_SUPPORT */
#if defined(CONFIG_ESL_NFC_SUPPORT)
#include "esl_nfc_impl.h"
#endif /* CONFIG_ESL_NFC_SUPPORT */

/* twis ... */
#include <nrfx_twis.h>

#define TWI_BUF_SZ    16
/** @brief Symbol specifying pin number of slave SCL. */
#define SLAVE_SCL_PIN 25

/** @brief Symbol specifying pin number of slave SDA. */
#define SLAVE_SDA_PIN 24
#define TWIS_INST_IDX 1
static nrfx_twis_t twis_inst = NRFX_TWIS_INSTANCE(TWIS_INST_IDX);

/** @brief TWIS recieve buffer for a message from TWIM. */
static uint8_t twis_rx_buffer[TWI_BUF_SZ];
static uint8_t twis_addr = 0xff;

/* twis ... */
#define WITH_TWIM // define to enable onboard twim, will need to jumper sda/scl
#ifdef WITH_TWIM
/* twim ... */
#include <nrfx_twim.h>

/** @brief Symbol specifying pin number of master SCL. */
#define MASTER_SCL_PIN 26

/** @brief Symbol specifying pin number of master SDA. */
#define MASTER_SDA_PIN 23

#define TWIM_INST_IDX 0
static nrfx_twim_t twim_inst = NRFX_TWIM_INSTANCE(TWIM_INST_IDX);

static uint8_t twim_tx_buffer[TWI_BUF_SZ];
static uint8_t twim_rx_buffer[TWI_BUF_SZ];

/* twim ...*/
#endif

LOG_MODULE_REGISTER(peripheral_esl, CONFIG_PERIPHERAL_ESL_LOG_LEVEL);

BT_ESL_DEF(esl_obj);

extern struct bt_conn *auth_conn;
static struct bt_esl_init_param init_param;

#if defined(AUTH_PASSKEY_MANUAL)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
	LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
}
#endif
static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d", addr, reason);
}

static void pairing_confirm(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	bt_conn_auth_pairing_confirm(conn);

	LOG_INF("Pairing confirmed: %s", addr);
}

void bond_deleted(uint8_t id, const bt_addr_le_t *peer)
{
	char addr[BT_ADDR_STR_LEN];

	bt_addr_le_to_str(peer, addr, sizeof(addr));
	LOG_INF("Bond deleted for %s, id %u", addr, id);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
#if defined(AUTH_PASSKEY_MANUAL)
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
#endif
	.pairing_confirm = pairing_confirm,
	.cancel = auth_cancel,
};
static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {.pairing_complete = pairing_complete,
							       .pairing_failed = pairing_failed,
							       .bond_deleted = bond_deleted};

static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (buttons & DK_BTN1_MSK) {
		printk("#DEBUG#FACTORYRESET:1\n");
		bt_esl_factory_reset();
	}
}

#if defined(CONFIG_ESL_WAKE_GPIO)
static void config_wakeup_gpio(void)
{
	nrf_gpio_cfg_input(CONFIG_ESL_WAKE_GPIO, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(CONFIG_ESL_WAKE_GPIO, NRF_GPIO_PIN_SENSE_LOW);
}
#endif /* CONFIG_ESL_WAKE_GPIO */

#if defined(CONFIG_ESL_SHIPPING_MODE)

static void system_off(void)
{
#if defined(CONFIG_ESL_SHIPPING_WAKE_BY_NFC)
#define WAKEUP_MSG "Approach an NFC reader to wake up.\n"
#elif defined(CONFIG_ESL_SHIPPING_WAKE_BY_BOTH)
#define WAKEUP_MSG "Approach an NFC reader or press the button to wake up.\n"
#elif defined(CONFIG_ESL_SHIPPING_WAKE_BY_BUTTON)
#define WAKEUP_MSG "Press the button to wake up.\n"
#else
#error "Invalid wake-up configuration"
#endif

	printk("Entering system off.\n");
	printk("%s", WAKEUP_MSG);

#if defined(CONFIG_ESL_POWER_PROFILE)
	/* Get rid of this by using user own driver */
	display_epd_onoff(EPD_POWER_OFF_IMMEDIATELY);
#endif
	sys_poweroff();
}

/**
 * @brief  Helper function for printing the reason of the last reset.
 * Can be used to confirm that NCF field actually woke up the system.
 */
static uint32_t print_reset_reason(void)
{
	uint32_t reas;

#if NRF_POWER_HAS_RESETREAS
	reas = nrf_power_resetreas_get(NRF_POWER);
	printk("rr 0x%08x ", reas);
	nrf_power_resetreas_clear(NRF_POWER, reas);
	if (reas & NRF_POWER_RESETREAS_NFC_MASK) {
		printk("Wake up by NFC field detect\n");
	} else if (reas & NRF_POWER_RESETREAS_RESETPIN_MASK) {
		printk("Reset by pin-reset\n");
	} else if (reas & NRF_POWER_RESETREAS_SREQ_MASK) {
		printk("Reset by soft-reset\n");
	} else if (reas & NRF_POWER_RESETREAS_OFF_MASK) {
		printk("Reset by Sense pin\n");
	} else if (reas) {
		printk("Reset by a different source (0x%08X)\n", reas);
	} else {
		printk("Power-on-reset\n");
	}
#else
	reas = nrf_reset_resetreas_get(NRF_RESET);
	printk("rr 0x%08x\n", reas);
	nrf_reset_resetreas_clear(NRF_RESET, reas);
	if (reas & NRF_RESET_RESETREAS_NFC_MASK) {
		printk("Wake up by NFC field detect\n");
	} else if (reas & NRF_RESET_RESETREAS_RESETPIN_MASK) {
		printk("Reset by pin-reset\n");
	} else if (reas & NRF_RESET_RESETREAS_SREQ_MASK) {
		printk("Reset by soft-reset\n");
	} else if (reas & NRF_RESET_RESETREA_OFF_MASK) {
		printk("Reset by Sense pin\n");
	} else if (reas) {
		printk("Reset by a different source (0x%08X)\n", reas);
	} else {
		printk("Power-on-reset\n");
	}
#endif
	return reas;
}

#endif /* CONFIG_ESL_SHIPPING_MODE */

static void configure_dk_button(void)
{
	int err;

	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}
}

#ifdef WITH_TWIM
static void twim_txrx()
{
	nrfx_err_t status;
	(void)status;

	uint8_t tx_fill = (k_uptime_get_32() & 0xff);
	memset(twim_rx_buffer, 0, sizeof(twim_rx_buffer));
	memset(twim_tx_buffer, tx_fill, sizeof(twim_tx_buffer));
	nrfx_twim_xfer_desc_t twim_xfer_desc =
		NRFX_TWIM_XFER_DESC_TXRX(twis_addr, twim_tx_buffer, sizeof(twim_tx_buffer),
					 twim_rx_buffer, sizeof(twim_rx_buffer));

	LOG_INF("TWIM start: addr %02x, send %02x...", twis_addr, tx_fill);

	status = nrfx_twim_xfer(&twim_inst, &twim_xfer_desc, 0);
	NRFX_ASSERT(status == NRFX_SUCCESS);
}
#endif

static int start_execute(void)
{
	int err;
	uint8_t own_addr_type;

	configure_dk_button();
	if (IS_ENABLED(CONFIG_BT_ESL_PTS)) {
		uint8_t pub_addr[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};

		bt_ctlr_set_public_addr(pub_addr);
		own_addr_type = BT_ADDR_LE_PUBLIC;
	} else {
		own_addr_type = BT_ADDR_LE_RANDOM;
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable error %d", err);
		return err;
	}

	LOG_INF("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	bt_id_set_scan_own_addr(true, &own_addr_type);
#if defined(CONFIG_ESL_NFC_SUPPORT)
	err = esl_msg_update();
	if (err) {
		LOG_WRN("esl_msg_update error %d", err);
	}
#endif /* CONFIG_ESL_NFC_SUPPORT */

	/* Assign hardware characteristics*/
	hw_chrc_init(&init_param);

	/* Assign hardware cb */
	init_param.cb.sensor_init = sensor_init;
	init_param.cb.sensor_control = sensor_control;
	init_param.cb.display_init = display_init;
	init_param.cb.display_control = display_control;
	init_param.cb.led_init = led_init;
	init_param.cb.led_control = led_control;
	init_param.cb.display_unassociated = display_unassociated;
	init_param.cb.display_associated = display_associated;
#if defined(CONFIG_CHARACTER_FRAMEBUFFER)
	init_param.cb.display_update_font = display_update_cfb;
	init_param.cb.display_clear_font = display_clear_cfb;
	init_param.cb.display_print_font = display_print_cfb;
#elif defined(CONFIG_BT_ESL_PAINT_LIB) || defined(CONFIG_BT_ESL_JF_PAINT_LIB)
	init_param.cb.display_update_font = display_update_paint;
	init_param.cb.display_clear_font = display_clear_paint;
	init_param.cb.display_print_font = display_print_paint;
#endif /* CONFIG_CHARACTER_FRAMEBUFFER */
#if defined(CONFIG_BT_ESL_IMAGE_AVAILABLE)
	init_param.cb.buffer_img = buffer_img;
	init_param.cb.open_image_from_storage = open_image_from_storage;
	init_param.cb.close_image_from_storage = close_image_from_storage;
	init_param.cb.write_img_to_storage = write_img_to_storage;
	init_param.cb.read_img_from_storage = read_img_from_storage;
	init_param.cb.read_img_size_from_storage = read_img_size_from_storage;
	init_param.cb.delete_imgs = delete_imgs_from_storage;

	err = ots_storage_init();
	if (err != 0) {
		LOG_ERR("Failed to init image storage (err:%d)\n", err);
		return err;
	}

#endif /* CONFIG_BT_ESL_IMAGE_AVAILABLE*/
#if defined(CONFIG_BT_ESL_VENDOR_SPECIFIC_SUPPORT)
	init_param.cb.vs_command_handler = vs_command_handler;
	init_param.cb.vs_response_handler = vs_response_handler;
#endif /* CONFIG_BT_ESL_VENDOR_SPECIFIC_SUPPORT */
	err = bt_esl_init(&esl_obj, &init_param);
	printk("bt_esl_init (err %d)\n", err);
	if (err != 0) {
		LOG_ERR("Failed to initialize ESL service (err: %d)", err);
		return err;
	}

	if (IS_ENABLED(CONFIG_BT_ESL_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization callbacks.");
			return err;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			LOG_ERR("Failed to register authentication information callbacks.");
			return err;
		}
	}

#if defined(CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION)
	printk("CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION %s\n", CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION);
#endif /* CONFIG_MCUBOOT_IMAGE_VERSION */

	for (;;) {
#ifdef WITH_TWIM
		// 5s txrx
		twim_txrx();
		k_sleep(SYS_TIMEOUT_MS(5000));
#else
		k_sleep(K_FOREVER);
#endif
	}
}

/**
 * @brief Function for handling TWIS driver events.
 *
 * @param[in] p_event Event information structure.
 */
static void twis_handler(nrfx_twis_evt_t const *p_event)
{
	nrfx_err_t status;
	(void)status;

	switch (p_event->type) {
	case NRFX_TWIS_EVT_WRITE_DONE:
		LOG_INF("--> Slave event: write done.");
		break;

	case NRFX_TWIS_EVT_READ_DONE:
		LOG_INF("--> Slave event: read done.");
		break;

	case NRFX_TWIS_EVT_WRITE_REQ:
		status = nrfx_twis_rx_prepare(&twis_inst, twis_rx_buffer, sizeof(twis_rx_buffer));
		NRFX_ASSERT(status == NRFX_SUCCESS);
		LOG_INF("--> Slave event: write request");
		break;

	case NRFX_TWIS_EVT_READ_REQ:
		status = nrfx_twis_tx_prepare(&twis_inst, twis_rx_buffer, sizeof(twis_rx_buffer));
		NRFX_ASSERT(status == NRFX_SUCCESS);
		LOG_INF("--> Slave event: read request");
		break;

	default:
		LOG_INF("--> SLAVE event: %d.", p_event->type);
	}
}

static void twis_init()
{
	nrfx_err_t status;
	(void)status; // not sure whats up with the assert macro

	// read the slave addr from the gpios
	uint8_t slave_addr = 0;
	uint8_t addrPins[] = {3, 28, 4, 2, 30, 31, 29, 27};
	for (int i = 0; i < 8; i++) {
		uint8_t pin = addrPins[i];
		nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_PULLDOWN);
		if (nrf_gpio_pin_read(pin) != 0) {
			slave_addr |= (1 << i);
		}
	}

	twis_addr = slave_addr;

	LOG_INF("TWIS init: slaveAddr 0x%02x, sda %d, scl %d", slave_addr, SLAVE_SDA_PIN,
		SLAVE_SCL_PIN);

	nrfx_twis_config_t twis_config =
		NRFX_TWIS_DEFAULT_CONFIG(SLAVE_SCL_PIN, SLAVE_SDA_PIN, slave_addr);
	// twis_config.sda_pull = NRF_GPIO_PIN_PULLUP;
	status = nrfx_twis_init(&twis_inst, &twis_config, twis_handler);
	NRFX_ASSERT(status == NRFX_SUCCESS);

	nrfx_twis_enable(&twis_inst);
}

#ifdef WITH_TWIM
/**
 * @brief Function for handling TWIM driver events.
 *
 * @param[in] p_event   Event information structure.
 * @param[in] p_context General purpose parameter set during initialization of the TWIM.
 *                      This parameter can be used to pass additional information to the
 *                      handler function. In this example @p p_context is used to pass address
 *                      of TWI transfer descriptor structure.
 */
static void twim_handler(nrfx_twim_evt_t const *p_event, void *p_context)
{
	// nrfx_twim_xfer_desc_t *twim_desc = p_context;
	if (p_event->type == NRFX_TWIM_EVT_DONE) {
		LOG_INF("--> Master event: done - transfer completed");
		LOG_INF("Content of master RX buffer: %02x...", twim_rx_buffer[0]);
		// }
	} else {
		LOG_INF("--> MASTER handler, event: %d.", p_event->type);
	}
}

void twim_init(void)
{
	nrfx_err_t status;
	(void)status;

	LOG_INF("TWIM init: sda %d, scl %d", MASTER_SDA_PIN, MASTER_SCL_PIN);
	nrfx_twim_xfer_desc_t twim_xfer_desc =
		NRFX_TWIM_XFER_DESC_TX(twis_addr, twim_tx_buffer, sizeof(twim_tx_buffer));
	nrfx_twim_config_t twim_config = NRFX_TWIM_DEFAULT_CONFIG(MASTER_SCL_PIN, MASTER_SDA_PIN);
	status = nrfx_twim_init(&twim_inst, &twim_config, twim_handler, &twim_xfer_desc);
	NRFX_ASSERT(status == NRFX_SUCCESS);

	nrfx_twim_enable(&twim_inst);
}
#endif

int main(void)
{
#ifdef WITH_TWIM
	IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TWIM_INST_GET(TWIM_INST_IDX)), IRQ_PRIO_LOWEST,
		    NRFX_TWIM_INST_HANDLER_GET(TWIM_INST_IDX), 0, 0);
#endif

	IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TWIS_INST_GET(TWIS_INST_IDX)), IRQ_PRIO_LOWEST,
		    NRFX_TWIS_INST_HANDLER_GET(TWIS_INST_IDX), 0, 0);

	twis_init(); // twis first to assign the static addr

#ifdef WITH_TWIM
	twim_init();
#endif

#if defined(CONFIG_ESL_NFC_SUPPORT)
	int err;

	err = esl_nfc_init();
	if (err) {
		LOG_WRN("esl_nfc_init error %d", err);
	}
#endif /* CONFIG_ESL_NFC_GREETING */

#if defined(CONFIG_ESL_SHIPPING_MODE)
	uint32_t rr = print_reset_reason();

#if NRF_POWER_HAS_RESETREAS
	if ((rr & NRF_POWER_RESETREAS_NFC_MASK) || (rr & NRF_POWER_RESETREAS_OFF_MASK)) {
		return start_execute();
#else
	if ((rr & NRF_RESET_RESETREAS_NFC_MASK) || (rr & NRF_RESET_RESETREAS_OFF_MASK)) {
		return start_execute();
#endif /* NRF_POWER_HAS_RESETREAS */
	} else {
#if defined(CONFIG_ESL_WAKE_GPIO)
		config_wakeup_gpio();
#endif /* CONFIG_ESL_WAKE_GPIO */
		system_off();
	}
#else

#ifdef WITH_TWIM
	twim_txrx();
#endif

	return start_execute();
#endif /* CONFIG_ESL_SHIPPING_MODE */
}
