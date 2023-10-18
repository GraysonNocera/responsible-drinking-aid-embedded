#include "USART.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>


void init_usart5() {
    // TODO
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //enable GPIOC
    RCC->AHBENR |= RCC_AHBENR_GPIODEN; //enable GPIOD

    GPIOC->MODER&= ~0x03000000; //clear PC12
    GPIOC->MODER|=0x02000000; //set pc12 alt func

    GPIOD->MODER&= ~0x0000000C; //clear PD2
    GPIOD->MODER|=0x00000020; //set PD2 alt func

    GPIOC->AFR[1]&=~(0xF<<16); //set alt funct PC12 to USART5_TX //alf2
    GPIOC->AFR[1]|=(0x2<<16); //set alt funct PC12 to USART5_TX //alf2

    GPIOD->AFR[0]&=~((0xF<<8));//set alt funct PD2 to USART5_RX // alf2
    GPIOD->AFR[0]|=((0x2<<8));//set alt funct PD2 to USART5_RX // alf2

    RCC->APB1ENR|=RCC_APB1ENR_USART5EN; //enable USART5 rcc clock

    USART5->CR1&=~USART_CR1_UE; //turn off usart5 enable bit

    USART5->CR1&=~(0x1<<28); //word length 8 but m1=0
    USART5->CR1&=~USART_CR1_M; //word length 8 but m0=0

    USART5->CR1&=~(USART_CR1_PCE); //turn off parity
    USART5->CR2&=~(0x3<<12); //stop bit set to 1 reg=00
    USART5->CR1&=~(USART_CR1_OVER8); //oversample bit set to 16
    USART5->BRR=0x1388; //set baud rate, the clock speed divided by this num(9600)

    USART5->CR1|=USART_CR1_RE|USART_CR1_TE; //enable RE and TE

    USART5->CR1|=USART_CR1_UE; //turn on usart5 enable bit

    while((!(USART5->ISR& USART_ISR_REACK)) && (!(USART5->ISR& USART_ISR_TEACK)));


    //wait until ISR RXE and TXE are set
}


void USART5_SendChar(char c) {
    while (!(USART5->ISR & USART_ISR_TXE)); // Wait until TX buffer is empty
    USART5->TDR = c;
}

void USART5_SendByte(uint16_t b) {
    // Send the upper byte
    USART5_SendChar((char)(b >> 8)+48);
    USART5_SendChar((char)(b & 0xFF)+48);
}

void USART5_SendString(const char* str) {
    while (*str) {
        USART5_SendChar(*str++);
    }
}

int init_HM19(void){
    USART5_SendString("AT");
    //USART5_SendString("AT+RENEW");
    //USART5_SendString("fhjsdhjkhfjewhfjkhewjkdhjewkhdjkshadkjhjdhsjkhnnnnnnnndjsadhfjkdshfewui    hsdjhew dewhjd edj wedhjwehd wedkj ewdj ewdhwejhdjew dewhd wehd jkewhd ewdhwe dhewj dhwejdhlakjhcuihcue cu cuih cuhecuihechdiuch dsuich uidsch uidhciusdahciuhewiuwe iweibewbdiewbdwebdbewdiubweduihew idh we chsd jchjsdjkcjdshchjds c hdchjdshcjhds c hjdsch whedh ewhdgewhdgh ew gdghewdgh gewhdg hwegh dhgewgh dgh ewdgh egwhdghewdgewghdghewgh dgewghdgh ewdghewgdweghdgh ewhdhg  gdhwegh d ghewghdweghdgh weghdgewdgwehdsdjwejdioewdjoiewo djddajf jewf ewfe foiewfioefijewfjiojioef fe jjwe jkewjf jew jkf jwe jfwe fwenfnewndndnwejkdewkhwlfhjefkjwe fhew fhuqewohfewhifweui fdwjkfhsdhfiuwhf eurgfeufbyeci bcuhebcyubcyreiucberucberucberybcyuer c jercued bcuy erbce cyer c ercbe cwecgedcjhsgd cdjh hejg fjhergf jehgwfhgdcb cyer cbuyecyuer uycbue bcuye bcuyer cbuyberucyb eurcb uycb ue cbue cbuye bcure cuyer cbu ercyub eruc ber cbuer cbuercbuyecbuyecbue cubj hdsjcd hcjb hcbreu bcuyer cbkjsedbchjs buyfr ekaufewi fyuewrufe fhbdsmhbcj hebe fuyerfuyer fvdjhfe erfguefer gffhrefhjsdhjkhfjewhfjkhewjkdhjewkhdjkshadkjhjdhsjkhdjsadhfjkdshfewui    hsdjhew dewhjd edj wedhjwehd wedkj ewdj ewdhwejhdjew dewhd wehd jkewhd ewdhwe dhewj dhwejdhlakjhcuihcue cu cuih cuhecuihechdiuch dsuich uidsch uidhciusdahciuhewiuwe iweibewbdiewbdwebdbewdiubweduihew idh we chsd jchjsdjkcjdshchjds c hdchjdshcjhds c hjdsch whedh ewhdgewhdgh ew gdghewdgh gewhdg hwegh dhgewgh dgh ewdgh egwhdghewdgewghdghewgh dgewghdgh ewdghewgdweghdgh ewhdhg  gdhwegh d ghewghdweghdgh weghdgewdgwehdsdjwejdioewdjoiewo djddajf jewf ewfe foiewfioefijewfjiojioef fe jjwe jkewjf jew jkf jwe jfwe fwenfnewndndnwejkdewkhwlfhjefkjwe fhew fhuqewohfewhifweui fdwjkfhsdhfiuwhf eurgfeufbyeci bcuhebcyubcyreiucberucberucberybcyuer c jercued bcuy erbce cyer c ercbe cwecgedcjhsgd cdjh hejg fjhergf jehgwfhgdcb cyer cbuyecyuer uycbue bcuye bcuyer cbuyberucyb eurcb uycb ue cbue cbuye bcure cuyer cbu ercyub eruc ber cbuer cbuercbuyecbuyecbue cubj hdsjcd hcjb hcbreu bcuyer cbkjsedbchjs buyfr ekaufewi fyuewrufe fhbdsmhbcj hebe fuyerfuyer fvdjhfe erfguefer gffhreghjg jyg hfhgfgh f ghfffdgffgdfg ddfg fg df");

    TIM2_delayOneSecond();
    TIM2_delayOneSecond();
    //get device name
   // USART5_SendString("AT+RESET");
    TIM2_delayOneSecond();
    //get device address
    USART5_SendString("AT+ADDR?");
    TIM2_delayOneSecond();
    //set advertising type
    USART5_SendString("AT+ADTY0");
    TIM2_delayOneSecond();
    //get device mode
    USART5_SendString("AT+MODE?");
    TIM2_delayOneSecond();
    //set role as peripheral
    USART5_SendString("AT+ROLE0");
    TIM2_delayOneSecond();
    //set so device starts working as soon as boot
    USART5_SendString("AT+IMME0");
    TIM2_delayOneSecond();

    //Reset advertising beacon
    USART5_SendString("AT+IBEA0");
    TIM2_delayOneSecond();

    //Turn on advertising beacon
    USART5_SendString("AT+IBEA1");
    TIM2_delayOneSecond();

    //get last connected device address
    USART5_SendString("AT+RADD?");
    TIM2_delayOneSecond();
////
//    //unneeded when device in IMME0, if in IMME1 this command will tell module to start working.
    USART5_SendString("AT+START");
    TIM2_delayOneSecond();
    USART5_SendString("AT+NAME?");
    TIM2_delayOneSecond();

    USART5_SendString("AT+MODE1");
    TIM2_delayOneSecond();
    USART5_SendString("AT+PWRM1");
    TIM2_delayOneSecond();
    USART5_SendString("AT+PARI?");
    TIM2_delayOneSecond();
    USART5_SendString("AT+POWE?");
    TIM2_delayOneSecond();
    USART5_SendString("AT+STOP?");
    TIM2_delayOneSecond();
    USART5_SendString("AT+SAVE?");
    TIM2_delayOneSecond();

}
