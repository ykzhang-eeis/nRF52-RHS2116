#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "RHS2116.h"
#include "nrf_delay.h"

uint32_t get_rtc_counter(void)
{
    return NRF_RTC1->COUNTER;
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(get_rtc_counter);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

int main(void)
{
    spi_init();
    rhs2116_init();
    log_init();
	
	for(;;){
		rhs2116_sampling();
		nrf_delay_ms(100);
	}
}
