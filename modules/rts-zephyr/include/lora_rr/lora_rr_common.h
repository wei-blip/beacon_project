//
// Created by rts on 05.02.2022.
//

#ifndef RADIO_SIGNALMAN_LORA_RR_COMMON_H
#define RADIO_SIGNALMAN_LORA_RR_COMMON_H

#include <devicetree.h>
#include <device.h>
#include <errno.h>


#include <drivers/lora.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>

#include "message_format.h"
#include "indication_rr/indication.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Количество событий в массиве app_event.
 * */
#define APPLICATION_EVENTS_NUM 3

/*
 * Значения этого еnum'a равны значениям индексов массива событий app_event.
 * Желательно не менять порядок этих enum'ов, если все таки такая необходимость будет,
 * то необходимо изменить соответствующим образом порядок в массиве событий app_event.
 * Если необходимо добавить новые события, то достаточно просто добавить в конец enum'a ещё одно значение и
 * добавить в конец массива app_event ещё один сигнал, а также поменять значения APPLICATION_EVENTS_NUM
 * */
enum APP_EVENTS_s {
  EVENT_TX_MODE = 0,
  EVENT_PROC_RX_DATA,
  EVENT_RX_MODE
};

/*
 * SLOT_TIME_MSEC - время одного тайм слота
 * PERIOD_TIME_MSEC - период на который заводится таймер
 * DELAY_TIME_MSEC - задержка
 * DURATION_TIME_MSEC - время, которое должно пройти с момента приёма синхро-кадра
 * до момента перевода модема в режим приёмника.
 * */
#define SLOT_TIME_MSEC 266UL /* Time on receive(166ms) + DELAY_TIME_MSEC */
#define PERIOD_TIME_MSEC (CONFIG_NUMBER_OF_DEVICES*SLOT_TIME_MSEC)  /* Timer period (super frame time) */
#define DELAY_TIME_MSEC 100U
#define DURATION_TIME_MSEC (SLOT_TIME_MSEC*(CONFIG_CURRENT_DEVICE_NUM-1)) /* Started delay for each device */

/**
 * Common peripheral settings area begin
 * */
#define DEFAULT_RADIO_NODE DT_NODELABEL(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
             "No default LoRa radio specified in DT");

#define PWM_SOUND	DT_ALIAS(pwm_sound)

#if DT_NODE_HAS_STATUS(PWM_SOUND, okay)
#define PWM_CTLR	DT_PWMS_CTLR(PWM_SOUND)
#define PWM_CHANNEL	DT_PWMS_CHANNEL(PWM_SOUND)
#define PWM_FLAGS	DT_PWMS_FLAGS(PWM_SOUND)
#else
#error "Unsupported board: pwm-led0 devicetree alias is not defined"
#define PWM_CTLR	DT_INVALID_NODE
#define PWM_CHANNEL	0
#define PWM_FLAGS	0
#endif
/**
 * Common peripheral settings area end
 * */

/**
 * My thread ids begin
 * */
extern const k_tid_t app_task_id;
/**
 * My thread ids end
 * */


/**
 * Thread functions begin
 * */
[[noreturn]] void app_task(void);
/**
 * Thread functions end
 * */


#define QUEUE_LEN_IN_ELEMENTS 10

#define BUTTON_PRESSED_PERIOD_TIME_USEC 40000UL /* PWM period time */

#define INTERVAL_TIME_MS 100
/* Counts interval time for detected pressed */
/* Задержки для разных типов нажатий */
#define SHORT_PRESSED_MIN_VAL 4  /* 400 ms */
#define SHORT_PRESSED_MAX_VAL 10 /* 1000 ms */
#define MIDDLE_PRESSED_MIN_VAL (SHORT_PRESSED_MAX_VAL+1)  /* 1100 ms */
#define MIDDLE_PRESSED_MAX_VAL 20   /* 2000 ms */
#define LONG_PRESSED_MIN_VAL (MIDDLE_PRESSED_MAX_VAL+1) /* 2100 ms */

extern atomic_t alarm_is_active;

/**
 * Enum, typedefs and structs area begin
 * */

enum BUZZER_MODES_e {
  BUZZER_MODE_SINGLE = 1,
  BUZZER_MODE_CONTINUOUS,
  BUZZER_MODE_DING_DONG,
  BUZZER_MODE_IDLE
};

enum CONNECTION_QUALITY_RSSI {
    CONNECTION_QUALITY_RSSI_1 = -70,
    CONNECTION_QUALITY_RSSI_2 = -80,
    CONNECTION_QUALITY_RSSI_3 = -90,
    CONNECTION_QUALITY_RSSI_4 = -100,
    CONNECTION_QUALITY_RSSI_5 = -105,
    CONNECTION_QUALITY_RSSI_6 = -125,
    CONNECTION_QUALITY_RSSI_7 = -127,
    CONNECTION_QUALITY_RSSI_8 = -130
};
/**
 * Enum, typedefs and structs area end
 * */


/**
 * Extern variable declaration area begin
 * */

/* Indicate structure */
extern struct led_strip_indicate_s status_ind;
extern struct led_strip_indicate_s disable_indication;
extern struct led_strip_indicate_s middle_pressed_button_ind;
extern struct led_strip_indicate_s msg_send_good_ind;
extern struct led_strip_indicate_s msg_send_bad_ind;
extern struct led_strip_indicate_s msg_recv_ind;
extern struct led_strip_indicate_s alarm_ind;
/**
 * Extern variable declaration area end
 * */


/**
 * Function declaration area begin
 * */

/**
 *
 * */
void work_button_pressed_handler(struct k_work *item);
void work_button_pressed_handler_dev(struct gpio_dt_spec *irq_gpio);
void dwork_disable_ind_handler(struct k_work *item);
void periodic_timer_handler(struct k_timer *tim);

/**
 * @brief Инициализация и конфигурация сервисов ядра
 * @return Ничего
 */
void common_kernel_services_init();

/**
 * @brief Колбэк функция для тайм-аута по прийму и ошибки по приёму модема
 * @param dev     LoRa device
 *
 * @return Ничего
 */
void lora_receive_error_timeout(const struct device *dev);

/**
 * @brief Колбэк функция по успешному приёму модема
 * @param dev     LoRa device
 * @param data    Указатель на массив с принятыми данными
 * @param size    Размер принятых данных
 * @param rssi    rssi значение
 * @param snr     snr значение
 *
 * @return Ничего
 */
void lora_receive_cb(const struct device *dev, uint8_t *data, uint16_t size, int16_t rssi, int8_t snr);

/**
 * @brief Функция для фильтрации и преобразования принятых данных из массива в экземпляр структуры message_s
 * @param recv_data     Указатель на массив с принятыми данными
 * @param len           Длина массива с принятыми данными
 * @param rx_msg        Указатель на экземпляр структуры, поля которой будут заполнены
 * @param cur_dev_addr  Адрес текущего устройства (Данный адрес выбирается из enum'a DEVICE_ADDR_e)
 *
 * @return true - если сообщение не отфильтровалось, false в противном случае
 */
bool proc_rx_data(uint8_t *recv_data, size_t len, struct message_s *rx_msg, DEVICE_ADDR_e cur_dev_addr);

/**
 * @brief Функция для взятия из очереди экземпляра структуры message_s и преобразования его в массив
 * для передачи по радио
 * @param msgq          Указатель на очередь из которой будет взято сообщение
 * @param tx_data       Указатель на буффер для передачи по радио
 * @param len           Длина буффера для передачи по радио
 * @param tx_msg        Указатель на экземпляр структуры message_s которая будет преобразована
 * в буффер для передачи по радио
 *
 * @return true - если сообщение есть в очереди, false в противном случае
 */
bool proc_tx_data(k_msgq *msgq, uint8_t *tx_data, size_t len, struct message_s *tx_msg);

bool radio_rx_queue_is_empty();

/**
 * @brief Функция для взятия из очереди значения rssi
 * @param rssi  Указатель на переменную в которую будет записано значение rssi
 *
 * @return Ничего
 */
void get_rssi(int16_t *rssi);

/**
 * @brief Функция для вставки экземпляра сообщения в очередь для отправки по радио
 * @param msg   Указатель на экземпляр сообщения, которое будет вставлено в очередь
 * @param prio  Если true - сообщение для отправки будет вставлено в приоритетную очередь, в противном случае
 * в обычную очередь
 *
 * @return Ничего
 */
void set_msg(struct message_s *msg, bool prio);

/**
 * @brief Функция для запуска зуммера в указанном режиме
 * @param buzzer_mode   Режим, в котром требуется запустить зуммер
 *
 * @return Ничего
 */
void set_buzzer_mode(BUZZER_MODES_e buzzer_mode);

/**
 * @brief Функция для запуска work'a при возникновении прерывания на кнопке
 * @param dev   Указатель на структуру gpio, которая в дальнейшем будет анализироваться в work'e
 *
 * @return Ничего
 */
void button_irq_routine(struct gpio_dt_spec *dev);

/**
 * @brief Функция для запуска периодического таймера
 * @param duration   Время до первого прерывания таймера
 * @param period     Время между прерываниями после срабатывания первого
 *
 * @return Ничего
 */
void tim_start(k_timeout_t duration, k_timeout_t period);

/**
 * @brief Функция для вставки указателя на экземпляр структуры с параметрами индикации
 * @param ind           Указатель на указатель экзмепляра структуры led_strip_indicate_s с параметрами индикации
 * @param duration_min  Если не равно K_FOREVER, то будет запущен отложенный вызов, в котором индикация будет выключена,
 * в противном случае установленная индикация не выключится
 *
 * @return Ничего
 */
void set_ind(led_strip_indicate_s **ind, k_timeout_t duration_min);

/**
 * @brief Функция опроса событий массива app_event
 *
 * @return Индекс на элемент события в массиве
 */
int8_t wait_app_event();

/**
 * @brief Функция для перевода модема в режим передачи.
 *        После выполнения этой функции будет автоматически запущен приём
 * @param lora_dev  LoRa device
 * @param lora_cfg  структура с настройками радио
 *
 * @return Если больше нуля, то очереди(приоритетная и обычная) для отправки сообщений пусты
 *         Если равно нулю, то сообщение успешно передано
 *         Если меньше нуля, то сообшение не отправлено
 */
int32_t start_tx(const struct device *lora_dev, struct lora_modem_config* lora_cfg);

/**
 * @brief Функция для перевода модема в режим передатчика
 * @param lora_dev  LoRa device
 * @param lora_cfg  структура с настройками радио
 *
 * @return Ничего
 */
void start_rx(const struct device *lora_dev, struct lora_modem_config* lora_cfg);

/**
 * @brief Функция анализа принятого сообщения
 * @param rx_msg    Указатель на структуру принятого сообщения
 * @param dev_data  Указатель на данные характерные для каждого устройства
 *
 * @return Ничего
 */
void analysis_fun(struct message_s* rx_msg, void *dev_data);

/**
 * @brief Функция анализа принятого сообщения если поле direction установлено в response
 * @param rx_msg        Указатель на структуру принятого сообщения
 * @param tx_msg        Указатель на структуру сообщения для возможног ответа
 * @param strip_ind     Указатель на структуру с параметрами индикации
 *
 * @return Ничего
 */
void request_analysis(const struct message_s *rx_msg, struct message_s *tx_msg, struct led_strip_indicate_s *strip_ind);

/**
 * @brief Функция анализа принятого сообщения если поле direction установлено в response
 * @param rx_msg        Указатель на структуру принятого сообщения
 * @param tx_msg        Указатель на структуру сообщения для возможног ответа
 * @param strip_ind     Указатель на структуру с параметрами индикации
 *
 * @return Ничего
 */
void response_analysis(const struct message_s *rx_msg, struct message_s *tx_msg, struct led_strip_indicate_s *strip_ind);
/**
 * Function declaration area end
 * */

static inline uint8_t check_rssi(int16_t rssi)
{
    uint8_t leds_num = 0;
    if ( rssi >= CONNECTION_QUALITY_RSSI_1 ) {
        leds_num = 8;
        return leds_num;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_1) && (rssi >= CONNECTION_QUALITY_RSSI_2) ) {
        leds_num = 7;
        return leds_num;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_2) && (rssi >= CONNECTION_QUALITY_RSSI_3) ) {
        leds_num = 6;
        return leds_num;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_3) && (rssi >= CONNECTION_QUALITY_RSSI_4) ) {
        leds_num = 5;
        return leds_num;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_4) && (rssi >= CONNECTION_QUALITY_RSSI_5) ) {
        leds_num = 4;
        return leds_num;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_5) && (rssi >= CONNECTION_QUALITY_RSSI_6) ) {
        leds_num = 3;
        return leds_num;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_6) && (rssi >= CONNECTION_QUALITY_RSSI_7) ) {
        leds_num = 2;
        return leds_num;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_7) && (rssi >= CONNECTION_QUALITY_RSSI_8) ) {
        leds_num = 1;
        return leds_num;
    }
    else if ( rssi < CONNECTION_QUALITY_RSSI_8 ) {
        return leds_num;
    }
}

#ifdef __cplusplus
}
#endif

#endif //RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_COMMON_H
