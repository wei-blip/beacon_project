#ifndef LORA_RUSSIA_RAILWAYS_SRC_LIBS_DWM_RR_COMMON_H_
#define LORA_RUSSIA_RAILWAYS_SRC_LIBS_DWM_RR_COMMON_H_

#include <drivers/uwb/deca_device_api.h>
#include <drivers/uwb/deca_regs.h>
#include <drivers/uwb/dw1000.h>
#include <meshposition/meshposition.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Workers number */
#define NUMBER_OF_NODES 8
/* AVAILABLE_WORKERS value can't be greater them NUMBER_OF_NODES */
#define AVAILABLE_WORKERS NUMBER_OF_NODES

#define RESPONDER_ID 255

/* Transaction flags */
#define TX_DONE 1
#define RX_DONE 2
#define RX_TO 3
#define RX_ERR 4

/* For slave device sleeping */
#define RNG_DELAY_MS 500

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547.0

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps)
 * conversion factor.
 * 1 uus = 512 / 499.2 usec and 1 usec = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

#define FINAL_RX_TIMEOUT_UUS 10000

/**
 * Timeouts for responder(base station) begin
 * */
#define POLL_RX_TO_RESP_TX_DLY_UUS 1000
#define RESP_TX_TO_FINAL_RX_DLY_UUS 300

#define FINAL_RX_TIMEOUT_UUS 10000
/**
 * Timeouts for responder(base station) end
 * */

/**
 * Timeouts for initiator(workers) begin
 * */
#define POLL_TX_TO_RESP_RX_DLY_UUS 500
#define RESP_RX_TO_FINAL_TX_DLY_UUS 5000

#define RESP_RX_TIMEOUT_UUS 10000
/**
 * Timeouts for initiator(workers) end
 * */

enum DWM_DEVICES_e {
  INITIATOR = 0,
  RESPONDER
};

extern struct k_msgq msgq_dwm_dist;
extern struct k_msgq msgq_dwt_callback_data;
extern struct k_poll_signal sig_rx_dwm;
extern struct k_poll_signal sig_tx_dwm;
extern struct k_poll_event ev_dwm[2];

/**
 * Function declaration area begin (This function can be use only for double side twr(ds_twr))
 * */
/* Callback functions */
void tx_ok_cb(const dwt_cb_data_t *cb_data); /* TxDone */
void rx_ok_cb(const dwt_cb_data_t *cb_data); /* RxDone */
void rx_to_cb(const dwt_cb_data_t *cb_data); /* RxTimeout */
void rx_err_cb(const dwt_cb_data_t *cb_data); /* RxError */

bool check_correct_recv(void *expect_msg, size_t expect_size, size_t recv_size);

/* Responder functions */
/**
 * @brief Функция для приёма первого (стартового) сообщения от инициатора
 *        и соответсвтующих действий после него для запрашивающего устройства
 * @param atomic_twr_status  Указатель на переменную с текущим состоянием транзакции
 *
 * @return Ничего
 */
void resp_twr_1_poll_ds_twr(atomic_t *atomic_twr_status);

/**
 * @brief Функция для отправки второго сообщения инициатору
 *        и соответсвтующих действий после него для запрашивающего устройства
 * @param rx_poll_msg  Указатель на принятое в стартовое сосбщение
 * @param poll_rx_ts   Временная метка принятого стартового сообщения
 * @param tx_resp_msg  Указатель на сообщение для отправки
 * @param twr_status   Указатель на переменную с текущим состоянием транзакции
 *
 * @return Ничего
 */
void resp_twr_2_resp_ds_twr(msg_header_t *rx_poll_msg, uint64_t poll_rx_ts, msg_header_t *tx_resp_msg,
                            atomic_t *twr_status);

/**
 * @brief Функция для обработки принятого сообщения полученного в ответ на отправленное второе и
 *        определения расстояния до узла
 * @param final_msg     Указатель на принятое финальное сообщение
 * @param poll_rx_ts    Временная метка принятого стартового сообщения
 * @param resp_tx_ts    Временная метка отправленного второго сообщения
 * @param final_rx_ts   Временная метка принятого финального сообщения
 * @param dist_src      Указатель на массив с расстояниями до узлов
 * @param nodes         Указатель на массив с активными узлами
 *
 * @return Ничего
 */
void resp_final_msg_poll_ds_twr(msg_twr_final_t *final_msg, uint64_t poll_rx_ts, uint64_t resp_tx_ts,
                                uint64_t final_rx_ts, double *dist_src, bool *nodes);

/* Initiator functions */
/**
 * @brief Функция для отправки стартовго сообщения запросчику
 * @param tx_poll_msg           Указатель на отправляемое сообщение
 * @param atomic_twr_status     Указатель на переменную с текущим состоянием транзакции
 *
 * @return Ничего
 */
void init_twr_1_poll_ds_twr(msg_header_t *tx_poll_msg, atomic_t *atomic_twr_status);

/**
 * @brief Функция для отправки финального сообщения запросчику
 * @param dwm_dev               DWM device
 * @param tx_poll_msg           Указатель на уже отправленное стартовое сообщение
 * @param atomic_twr_status     Указатель на переменную с текущим состоянием транзакции
 * @param cb_data               Структура колбэк данных (Не знаю что написать)
 *
 * @return Ничего
 */
void init_twr_2_resp_ds_twr(const struct device *dwm_dev, msg_header_t *tx_poll_msg, atomic_t *atomic_twr_status,
                            dwt_cb_data_t *cb_data);
/**
* Function declaration area end
* */

extern const k_tid_t dwm_task_id;

/**
 * Thread functions area begin
 * */
[[noreturn]] void dwm_task();
/**
* Thread functions area end
* */
#ifdef __cplusplus
}
#endif

#endif
