#include <stm32f4xx.h>
#define MASK(x) (1UL << (x))
#define mode_read 128



volatile int physical_address;
volatile int data;
volatile int register_led_number;
volatile int leds_array[4]; 
 
 
void UART_Transmit(char digit){
 USART2->DR = digit;
 while(!READ_BIT(USART2->SR, USART_SR_TC)){}
}

void update_leds(){
		volatile int selected = 0;
		selected |= leds_array[0] << 4;
		selected |= leds_array[1] << 5;
		selected |= leds_array[2] << 6;
		selected |= leds_array[3] << 7;
		GPIOC->ODR  = 0;
		GPIOC->ODR |= selected;
} 
 
 
void state_manager(){
	if (physical_address == 1 ){
		leds_array[register_led_number] = data;
		update_leds();
	}
	
} 


void send_data(int led_num){
		volatile char phy_address  = 1;
		volatile char register_led = led_num + 128;
		volatile char data_led     = leds_array[led_num];
		UART_Transmit(phy_address);
		UART_Transmit(register_led);
		UART_Transmit(data_led);
}
 
 
void delay(uint16_t amt){
 volatile uint16_t i;
 for(i = 0; i < amt; i++){}
}





 



void EXTI0_IRQHandler(void){
	
	EXTI-> PR |= MASK(0);
	NVIC_ClearPendingIRQ(EXTI0_IRQn);

	if(GPIOC->IDR & MASK(0)){		
		GPIOA->ODR = 1;
		leds_array[0] ^=1 ;
		send_data(0);
		GPIOA->ODR = 0;
		update_leds();
	}

		
	

}

void EXTI1_IRQHandler(void){
	
	EXTI-> PR |= MASK(0);
	NVIC_ClearPendingIRQ(EXTI1_IRQn);

		if( GPIOC->IDR & MASK(1)){
			GPIOA->ODR = 1;
			leds_array[1] ^=1 ;
			send_data(1);
			GPIOA->ODR = 0;
			update_leds();
		}		

}


void EXTI2_IRQHandler(void){
	
	EXTI-> PR |= MASK(0);
	NVIC_ClearPendingIRQ(EXTI2_IRQn);

	if( GPIOC->IDR & MASK(2)){
			GPIOA->ODR = 1;
			leds_array[2] ^= 1 ;
			send_data(2);
			GPIOA->ODR = 0;
			update_leds();

	}
	

}

void EXTI3_IRQHandler(void){
	
	EXTI-> PR |= MASK(0);
	NVIC_ClearPendingIRQ(EXTI3_IRQn);

	if(GPIOC->IDR & MASK(3)){
			GPIOA->ODR = 1;
			leds_array[3] ^=1 ;
			send_data(3);
			GPIOA->ODR = 0;
		  update_leds();
	}
	

}


void irq0_init(){
	SYSCFG-> EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC;
	EXTI->IMR |= (MASK(0));
	EXTI->RTSR |= (MASK(0));
	__enable_irq();
	NVIC_SetPriority(EXTI0_IRQn,0);
	NVIC_ClearPendingIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI0_IRQn);

}


void irq1_init(){
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PC;	
	EXTI->IMR |= (MASK(1));												
	EXTI->RTSR |= (MASK(1));
	NVIC_SetPriority(EXTI1_IRQn,0);
	NVIC_ClearPendingIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);

}


void irq2_init(){
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PC;	
	EXTI->IMR |= (MASK(2));												
	EXTI->RTSR |= (MASK(2));
	NVIC_SetPriority(EXTI2_IRQn,0);
	NVIC_ClearPendingIRQ(EXTI2_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);

}

void irq3_init(){
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PC;	
	EXTI->IMR |= (MASK(3));												
	EXTI->RTSR |= (MASK(3));
	NVIC_SetPriority(EXTI3_IRQn,3);
	NVIC_ClearPendingIRQ(EXTI3_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);
}






void GPIO_init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	// PA0
	GPIOA->MODER |= 1;
	
	
	// PA5, PA7 for SPI1 MOSI and SCLK
	GPIOA->MODER &= ~((3 << 10) | (3 << 14));
	GPIOA->MODER |= ((2 << 10) | (2 << 14)); // AF MODE
	GPIOA->AFR[0] &= ~0xF0F00000;
	GPIOA->AFR[0] |= 0x50500000;
	// PA4 as GPIO output for SPI slave select
	GPIOA->MODER &= ~(3 << 8);
	GPIOA->MODER |= (1 << 8);
}

void SPI1_init(void)
{
	GPIO_init();
	SPI1->CR1 = 0;
	SPI1->CR1 = 0x31C; // 9600 Baud rate, 8-bit // LSBFIRST
	SPI1->CR2 = 0;
	SPI1->CR1 |= SPI_CR1_SPE;
}


void USART2_init (void) { 
 RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  /* enable GPIOA clock */ 
 RCC->APB1ENR |= 0x20000;        /* enable USART2 clock */ 
 /* Configure PA2 for USART2_TX */ 
 GPIOA->OSPEEDR |= 0x20; 
 GPIOA->OSPEEDR |= 0x80; 
 GPIOA->AFR[0] &= ~0x0FF00; 
 GPIOA->AFR[0] |= 0x07700; /* alt7 for USART2 */ 
 GPIOA->MODER  &= ~0x00F0; 
 GPIOA->MODER  |= 0x00A0; /* enable alternate function for PA2,PA3*/ 
 USART2->BRR = 0x0683; /* 9600 baud @ 16 MHz */ 
 USART2->CR1 |= 0x0008; /* enable Tx, 8-bit data */ 
 USART2->CR1 |= 0x0004; /* enable Rx, 8-bit data */ 
 //USART2->CR1 |= USART_CR1_TCIE; 
 USART2->CR2 |= (2 << 12); /* 1 stop bit */ 
 USART2->CR3 = 0x0000; /* no flow control */ 
 USART2->CR1 |= 0x2000; /* enable USART2 */ 
 
} 
 
char USART2_receive(){ 
 while (!READ_BIT(USART2->SR, USART_SR_RXNE)){} 
		physical_address = (int)(USART2->DR); 
 while (!READ_BIT(USART2->SR, USART_SR_RXNE)){} 
		register_led_number = (int)(USART2->DR);  
	while (!READ_BIT(USART2->SR, USART_SR_RXNE)){} 
		data = (int)(USART2->DR); 
 return 't'; 
}

int char_to_int(char d){
	if (d=='0'){
			return 0;
	}
	else if (d == '1')
		return 1;
	else if (d == '2')
		return 2;
	else if (d == '3')
		return 3;

}






int main(){

		
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;	
		RCC-> APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOC->MODER = 0x5500;
		irq0_init();
		irq1_init();
		irq2_init();
		irq3_init();
		SPI1_init();
		USART2_init();
		GPIOC->ODR = 0;
		GPIOA->ODR = 0; //control bus
    while(1) {
			
			USART2_receive();
			state_manager();
		
    }
	
}
