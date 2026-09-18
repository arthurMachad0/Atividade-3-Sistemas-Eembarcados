#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/dac_oneshot.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *ETIQUETA_LOG = "dac_waveform";

/* ============================ Configuracao de pinos ============================ */
#define PINO_BOTAO_ONDA        GPIO_NUM_32   /* Botao 1: troca a forma de onda   */
#define PINO_BOTAO_AMPLITUDE   GPIO_NUM_33   /* Botao 2: altera a amplitude      */
#define PINO_BOTAO_FREQUENCIA  GPIO_NUM_27   /* Botao 3: altera a frequencia     */

/* ============================ Parametros da onda ============================ */
#define NUM_AMOSTRAS   100   /* quantidade de amostras que formam um ciclo de onda */

/* Faixa permitida para a amplitude, em porcentagem da escala total do DAC */
#define AMPLITUDE_MIN_PORCENTO    20
#define AMPLITUDE_MAX_PORCENTO    100
#define AMPLITUDE_PASSO_PORCENTO  10   /* quanto cada clique do botao 2 soma */

/* Faixa permitida para a frequencia da onda, em Hz */
#define FREQUENCIA_MIN_HZ    1
#define FREQUENCIA_MAX_HZ    100
#define FREQUENCIA_PASSO_HZ  5   /* quanto cada clique do botao 3 soma */

/* Parametros do antirrebote (debounce) dos botoes */
#define ANTIRREBOTE_US               200000  /* 200 ms: intervalo minimo entre dois cliques aceitos */
#define ANTIRREBOTE_CONFIRMACAO_MS   20      /* espera antes de confirmar se o pino continua pressionado */

/* Tipos de onda que sabemos gerar, em ordem de alternancia do botao 1 */
typedef enum {
    ONDA_SENOIDAL = 0,
    ONDA_TRIANGULAR,
    ONDA_DENTE_SERRA,
    ONDA_QUADRADA,
    ONDA_QUANTIDADE   /* nao e uma onda de verdade, serve so para contar quantas existem */
} tipo_onda_t;

static const char *nomes_ondas[ONDA_QUANTIDADE] = {
    "Senoidal", "Triangular", "Dente de serra", "Quadrada"
};

static uint8_t tabela_senoidal[NUM_AMOSTRAS];
static uint8_t tabela_triangular[NUM_AMOSTRAS];
static uint8_t tabela_dente_serra[NUM_AMOSTRAS];
static uint8_t tabela_quadrada[NUM_AMOSTRAS];

/* Vetor de ponteiros: permite escolher a tabela ativa so trocando um indice,
 * sem precisar copiar dados nem usar um "switch" gigante em tempo real. */
static const uint8_t *tabelas_onda[ONDA_QUANTIDADE];

static void montar_tabelas_onda(void)
{
    for (int i = 0; i < NUM_AMOSTRAS; i++) {
        /* fase vai de 0.0 (inicio do ciclo) a quase 1.0 (fim do ciclo) */
        float fase = (float)i / (float)NUM_AMOSTRAS;

        /* Onda senoidal: um seno completo, deslocado e escalado para caber
         * na faixa 0-255 em vez de -1.0 a 1.0 */
        tabela_senoidal[i] = (uint8_t)(127.5f + 127.5f * sinf(2.0f * (float)M_PI * fase));

        /* Dente de serra: sobe em linha reta de 0 ate 255 e recomeca do zero */
        tabela_dente_serra[i] = (uint8_t)(255.0f * fase);

        /* Triangular: sobe em linha reta na primeira metade do ciclo e desce
         * em linha reta na segunda metade */
        if (fase < 0.5f) {
            tabela_triangular[i] = (uint8_t)(255.0f * (fase / 0.5f));
        } else {
            tabela_triangular[i] = (uint8_t)(255.0f * (1.0f - (fase - 0.5f) / 0.5f));
        }

        /* Quadrada: metade do ciclo no nivel maximo, metade no nivel minimo */
        tabela_quadrada[i] = (fase < 0.5f) ? 255 : 0;
    }

    /* Associa cada indice do enum tipo_onda_t a sua respectiva tabela */
    tabelas_onda[ONDA_SENOIDAL]    = tabela_senoidal;
    tabelas_onda[ONDA_TRIANGULAR]  = tabela_triangular;
    tabelas_onda[ONDA_DENTE_SERRA] = tabela_dente_serra;
    tabelas_onda[ONDA_QUADRADA]    = tabela_quadrada;
}


static SemaphoreHandle_t mutex_estado;

static tipo_onda_t indice_onda        = ONDA_SENOIDAL;
static uint32_t    amplitude_porcento = AMPLITUDE_MAX_PORCENTO;
static uint32_t    frequencia_hz      = 10;

/* ============================ DAC e timer de amostragem ============================ */
static dac_oneshot_handle_t canal_dac;        
static esp_timer_handle_t   timer_amostragem;  
static TaskHandle_t         tarefa_dac_handle; 

static void aplicar_frequencia(uint32_t nova_frequencia_hz)
{
    uint64_t periodo_us = 1000000ULL / ((uint64_t)nova_frequencia_hz * NUM_AMOSTRAS);
    if (periodo_us < 50) {
        periodo_us = 50;
    }

    esp_timer_stop(timer_amostragem);
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_amostragem, periodo_us));
}

static void callback_timer(void *arg)
{
    xTaskNotifyGive(tarefa_dac_handle);
}

static void tarefa_dac(void *arg)
{
    size_t indice_amostra = 0;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        xSemaphoreTake(mutex_estado, portMAX_DELAY);
        const uint8_t *tabela_atual = tabelas_onda[indice_onda];
        uint8_t  amostra_bruta      = tabela_atual[indice_amostra];
        uint32_t amplitude_atual    = amplitude_porcento;
        xSemaphoreGive(mutex_estado);

        int valor_centralizado = (int)amostra_bruta - 128;
        int valor_escalado     = (valor_centralizado * (int)amplitude_atual) / 100;
        int valor_final        = 128 + valor_escalado;

        if (valor_final < 0)   valor_final = 0;
        if (valor_final > 255) valor_final = 255;

        dac_oneshot_output_voltage(canal_dac, (uint8_t)valor_final);
      
        indice_amostra = (indice_amostra + 1) % NUM_AMOSTRAS;
    }
}

/* ============================ Botoes (GPIO + interrupcao + fila) ============================ */
static QueueHandle_t gpio_evt_queue;

static const gpio_num_t pinos_botoes[3] = {
    PINO_BOTAO_ONDA, PINO_BOTAO_AMPLITUDE, PINO_BOTAO_FREQUENCIA
};

static int indice_botao_a_partir_do_pino(uint32_t numero_pino)
{
    for (int i = 0; i < 3; i++) {
        if ((uint32_t)pinos_botoes[i] == numero_pino) {
            return i;
        }
    }
    return -1;
}

static void IRAM_ATTR tratador_interrupcao_botao(void *arg)
{
    uint32_t numero_pino = (uint32_t)(uintptr_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &numero_pino, NULL);
}

static void tratar_pressionamento_botao(int indice_botao)
{
    switch (indice_botao) {

        case 0: { /* Botao 1: troca a forma de onda */
            xSemaphoreTake(mutex_estado, portMAX_DELAY);
            indice_onda = (indice_onda + 1) % ONDA_QUANTIDADE;
            tipo_onda_t onda_selecionada = indice_onda;
            xSemaphoreGive(mutex_estado);

            ESP_LOGI(ETIQUETA_LOG, "Forma de onda: %s", nomes_ondas[onda_selecionada]);
            break;
        }

        case 1: { /* Botao 2: incrementa a amplitude (com efeito dente de serra) */
            xSemaphoreTake(mutex_estado, portMAX_DELAY);
            amplitude_porcento += AMPLITUDE_PASSO_PORCENTO;
            if (amplitude_porcento > AMPLITUDE_MAX_PORCENTO) {
                amplitude_porcento = AMPLITUDE_MIN_PORCENTO;
            }
            uint32_t nova_amplitude = amplitude_porcento;
            xSemaphoreGive(mutex_estado);

            ESP_LOGI(ETIQUETA_LOG, "Amplitude: %lu%%", (unsigned long)nova_amplitude);
            break;
        }

        case 2: { /* Botao 3: incrementa a frequencia (com efeito dente de serra) */
            xSemaphoreTake(mutex_estado, portMAX_DELAY);
            frequencia_hz += FREQUENCIA_PASSO_HZ;
            if (frequencia_hz > FREQUENCIA_MAX_HZ) {
                frequencia_hz = FREQUENCIA_MIN_HZ;
            }
            uint32_t nova_frequencia = frequencia_hz;
            xSemaphoreGive(mutex_estado);

            aplicar_frequencia(nova_frequencia);

            ESP_LOGI(ETIQUETA_LOG, "Frequencia: %lu Hz", (unsigned long)nova_frequencia);
            break;
        }

        default:
            break; /* nao deveria acontecer, mas evita comportamento indefinido */
    }
}

static void tarefa_botoes(void *arg)
{
    uint32_t numero_pino;
    int64_t  tempo_ultimo_clique[3] = { 0, 0, 0 };

    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &numero_pino, portMAX_DELAY)) {

            int indice_botao = indice_botao_a_partir_do_pino(numero_pino);
            if (indice_botao < 0) {
                continue; /* pino desconhecido, ignora */
            }

            int64_t agora = esp_timer_get_time();
            if (agora - tempo_ultimo_clique[indice_botao] < ANTIRREBOTE_US) {
                continue; /* clique aceito ha pouco tempo: descarta repique */
            }

            /* Espera um pouco e confere se o botao continua pressionado de
             * verdade antes de aceitar o clique. */
            vTaskDelay(pdMS_TO_TICKS(ANTIRREBOTE_CONFIRMACAO_MS));
            if (gpio_get_level(pinos_botoes[indice_botao]) != 0) {
                continue; /* pino ja voltou a nivel alto: foi so ruido */
            }

            tempo_ultimo_clique[indice_botao] = esp_timer_get_time();

            tratar_pressionamento_botao(indice_botao);
        }
    }
}

static void inicializar_botoes(void)
{
    gpio_config_t configuracao_botoes = {
        .pin_bit_mask = (1ULL << PINO_BOTAO_ONDA)
                       | (1ULL << PINO_BOTAO_AMPLITUDE)
                       | (1ULL << PINO_BOTAO_FREQUENCIA),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&configuracao_botoes));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(gpio_isr_handler_add(pinos_botoes[i], tratador_interrupcao_botao,
                                              (void *)(uintptr_t)pinos_botoes[i]));
    }
}

static void inicializar_dac(void)
{
    dac_oneshot_config_t configuracao_dac = {
        .chan_id = DAC_CHAN_0, /* GPIO25 */
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&configuracao_dac, &canal_dac));
}

/* ============================ Ponto de entrada do programa ============================ */
void app_main(void)
{

    mutex_estado    = xSemaphoreCreateMutex();
    gpio_evt_queue  = xQueueCreate(10, sizeof(uint32_t));

    montar_tabelas_onda();
    inicializar_dac();
    inicializar_botoes();
  
    xTaskCreate(tarefa_dac,    "tarefa_dac",    4096, NULL, 10, &tarefa_dac_handle);
    xTaskCreate(tarefa_botoes, "tarefa_botoes", 4096, NULL, 5,  NULL);

    const esp_timer_create_args_t configuracao_timer = {
        .callback = &callback_timer,
        .name     = "timer_amostragem_dac",
    };
    ESP_ERROR_CHECK(esp_timer_create(&configuracao_timer, &timer_amostragem));
    aplicar_frequencia(frequencia_hz);

    ESP_LOGI(ETIQUETA_LOG,
             "Sistema iniciado. Onda: %s | Amplitude: %lu%% | Frequencia: %lu Hz",
             nomes_ondas[indice_onda],
             (unsigned long)amplitude_porcento,
             (unsigned long)frequencia_hz);
}
