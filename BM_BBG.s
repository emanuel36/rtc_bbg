//Registers
.equ CM_PER_GPIO1_CLKCTRL, 0x44e000AC
.equ GPIO1_OE, 0x4804C134
.equ GPIO1_SETDATAOUT, 0x4804C194
.equ GPIO1_CLEARDATAOUT, 0x4804C190
.equ WDT_BASE, 0x44E35000
.equ UART0_BASE, 0x44E09000
.equ RTC_BASE, 0x44E3E000
.equ CM_RTC_RTC_CLKCTRL, 0x44E00800
.equ CM_RTC_CLKSTCTRL, 0x44E00804

_start:
    /* init */
    mrs r0, cpsr
    bic r0, r0, #0x1F @ clear mode bits
    orr r0, r0, #0x13 @ set SVC mode
    orr r0, r0, #0xC0 @ disable FIQ and IRQ
    msr cpsr, r0

    bl disable_wdt
    bl gpio_setup
    bl rtc_setup

/********************************************************
Escreve a hora nos registradores da hora
********************************************************/
write_time:
	bl print_string_hora
	bl recebe_digito
	lsl r1, r0, #4  //DIGITO 1
	ldr r0, =RTC_BASE
	mov r3, #0x8 
	add r3, r0, r3 //HORA
	str r1, [r3]

	bl recebe_digito
	mov r2, r0  //DIGITO 2
	orr r1, r1, r2
	str r1, [r3]

	ldr r0,=':'
	bl uart_putc

	bl recebe_digito
	lsl r1, r0, #4  //DIGITO 1
	ldr r0, =RTC_BASE
	mov r3, #0x4 
	add r3, r0, r3 //MINUTOS
	str r1, [r3]

    bl recebe_digito
	mov r2, r0 //DIGITO 2 
	orr r1, r1, r2
	str r1, [r3]

	ldr r0,=':'
	bl uart_putc

	bl recebe_digito
	lsl r1, r0, #4  //DIGITO 1
	ldr r0, =RTC_BASE
	mov r3, #0x0 
	add r3, r0, r3 //SEGUNDOS
	str r1, [r3]

	bl recebe_digito
	mov r2, r0  //DIGITO 2
	orr r1, r1, r2
	str r1, [r3]

/********************************************************
Escreve a data nos registradores da data
********************************************************/
write_data:
	ldr r0,='	'
	bl uart_putc
	
	bl print_string_data
	bl recebe_digito
	lsl r1, r0, #4  //DIGITO 1
	ldr r0, =RTC_BASE
	mov r3, #0xC 
	add r3, r0, r3 //DIA
	//str r1, [r3]

	bl recebe_digito
	mov r2, r0  //DIGITO 2
	orr r1, r1, r2
	str r1, [r3]

	ldr r0,='/'
	bl uart_putc

	bl recebe_digito
	lsl r1, r0, #4  //DIGITO 1
	ldr r0, =RTC_BASE
	mov r3, #0x10 
	add r3, r0, r3 //MES
	str r1, [r3]

    bl recebe_digito
	mov r2, r0 //DIGITO 2 
	orr r1, r1, r2
	str r1, [r3]

	ldr r0,='/'
	bl uart_putc

	bl recebe_digito
	lsl r1, r0, #4  //DIGITO 1
	ldr r0, =RTC_BASE
	mov r3, #0x14 
	add r3, r0, r3 //DIA
	str r1, [r3]

	bl recebe_digito
	mov r2, r0  //DIGITO 2
	orr r1, r1, r2
	str r1, [r3]

	bl print_quebra

/********************************************************
Ler os registradores da hora e imprime na tela 
********************************************************/
read_time: 
	bl print_string_hora

	ldr r0, =RTC_BASE
	ldr r1, [r0, #0x8] //Hora
	mov r0, r1, lsr #0x4
	add r0, r0, #0x30
	bl uart_putc
	and r0, r1, #0xF
	add r0, r0, #0x30
	bl uart_putc 

	ldr r0,=':'
	bl uart_putc

	ldr r0, =RTC_BASE
	ldr r1, [r0, #0x4] //Minutes
	mov r0, r1, lsr #0x4
	add r0, r0, #0x30
	bl uart_putc
	and r0, r1, #0xF
	add r0, r0, #0x30
	bl uart_putc

	ldr r0,=':'
	bl uart_putc

	ldr r0, =RTC_BASE
	ldr r1, [r0, #0x0] //Seconds
	mov r0, r1, lsr #0x4
	add r0, r0, #0x30
	bl uart_putc
	and r0, r1, #0xF
	add r0, r0, #0x30
	bl uart_putc
	
	cmp r1, r2
	blne blink
	mov r2, r1

/********************************************************
Ler os registradores da data e imprime na tela 
********************************************************/
read_data:
	mov r0, #0x9 //Tab
	bl uart_putc

	bl print_string_data

	ldr r0, =RTC_BASE
	ldr r1, [r0, #0xC] //Dia
	mov r0, r1, lsr #0x4
	add r0, r0, #0x30
	bl uart_putc
	and r0, r1, #0xF
	add r0, r0, #0x30
	bl uart_putc

	ldr r0,='/'
	bl uart_putc

	ldr r0, =RTC_BASE
	ldr r1, [r0, #0x10] //Mes
	mov r0, r1, lsr #0x4
	add r0, r0, #0x30
	bl uart_putc
	and r0, r1, #0xF
	add r0, r0, #0x30
	bl uart_putc

	ldr r0,='/'
	bl uart_putc

	ldr r0, =RTC_BASE
	ldr r1, [r0, #0x14] //Ano
	mov r0, r1, lsr #0x4
	add r0, r0, #0x30
	bl uart_putc
	and r0, r1, #0xF
	add r0, r0, #0x30
	bl uart_putc

	ldr r0, ='\r'
	bl uart_putc //Sobrescreve

	b read_time

/********************************************************
Configura o RTC 
********************************************************/
rtc_setup:
    ldr r0, =CM_RTC_CLKSTCTRL
    ldr r1, =0x2
    str r1, [r0]
    ldr r0, =CM_RTC_RTC_CLKCTRL
    str r1, [r0]

    /*Disable write protection*/
    ldr r0, =RTC_BASE
    ldr r1, =0x83E70B13
    str r1, [r0, #0x6c]
    ldr r1, =0x95A4F1E0
    str r1, [r0, #0x70]
    
    /* Select external clock*/
    ldr r1, =0x48
    str r1, [r0, #0x54]

    ldr r1, =0x4     /* interrupt every second */
    //ldr r1, =0x5     /* interrupt every minute */
    str r1, [r0, #0x48]

    /* Enable RTC */
    ldr r0, =RTC_BASE
    ldr r1, =0x01
    str r1, [r0, #0x40]   
    
    bx lr

/********************************************************
Recebe um dígito do teclado
********************************************************/
recebe_digito:
	stmfd sp!, {lr}
	bl uart_getc
	cmp r0, #0
	beq recebe_digito
	bl uart_putc	
	sub r0, r0, #0x30
	ldmfd sp!, {pc}

/********************************************************
Imprime r0
********************************************************/
uart_putc:
	stmfd sp!, {r0-r2,lr}
	ldr r1, =UART0_BASE

wait_tx_fifo_empty:
	ldr r2, [r1, #0x14]
	and r2, r2, #(1<<5)
	cmp r2, #0
	beq wait_tx_fifo_empty

	strb r0, [r1]
	ldmfd sp!, {r0-r2,pc}

/********************************************************
Recebe e salva em r0
********************************************************/
uart_getc:
    stmfd sp!,{r1-r2,lr}
    ldr     r1, =UART0_BASE

wait_rx_fifo:
    ldr r2, [r1, #0x14] 
    and r2, r2, #(1<<0)
    cmp r2, #0
    beq wait_rx_fifo

    ldrb  r0, [r1]
    ldmfd sp!,{r1-r2,pc}

/********************************************************
Print r3 in screen
********************************************************/
print_number:
	stmfd sp!,{r0-r2,lr}
	mov r2, #100

print_loop:
	mov r0, r2
	mov r1, #10
	bl div
	mov r2, r0
	mov r0, r3
	mov r1, r2
	bl div
	add r0, r0, #0x30
	bl uart_putc
	mov r3, r1
	cmp r2, #1
	bne print_loop
	ldmfd sp!,{r0-r2,pc}

/********************************************************
Print string in r3 in screen
********************************************************/
print_string:
    stmfd sp!,{r0-r2,lr}
print:
    ldrb r0,[r3],#1
    and r0, r0, #0xff
    cmp r0, #0
    beq end_print
    bl uart_putc
    b print
    
end_print:
    ldmfd sp!,{r0-r2,pc}

/********************************************************
Divisão
r0 = r0 / r1
r1 = resto
********************************************************/
div:
	stmfd sp!,{r2-r3,lr}
    mov r2, r0 //num
    mov r3, r1 //den
    mov r0, #0
div_loop:
    cmp r2, r3
    blt fim_div 
    sub r2, r2, r3
    add r0, r0, #1
    b div_loop
fim_div:
	mov r1, r2
    ldmfd sp!,{r2-r3,pc}
   
/********************************************************
  Blink Leds
********************************************************/
blink:
	stmfd sp!, {lr}
	bl set_gpio
	bl delay
	bl clear_gpio
	bl delay
	ldmfd sp!, {pc}

/********************************************************
  Delay not precise
********************************************************/
delay:
	stmfd sp!, {r1,lr}
    ldr r1, =0x2FFFFFF
wait:
    sub r1, r1, #0x1
    cmp r1, #0
    bne wait
	ldmfd sp!, {r1,pc}

/********************************************************
  Imprime mensagens especificas
********************************************************/
print_string_hora:
	stmfd sp!, {lr}
	adr r3, hora
	bl print_string	
	ldmfd sp!, {pc}

print_string_data:
	stmfd sp!, {lr}
	adr r3, data 
	bl print_string	
	ldmfd sp!, {pc}

print_quebra:
	stmfd sp!, {lr}
	adr r3, quebra_linha
	bl print_string	
	ldmfd sp!, {pc}

/********************************************************
  Set gpio value
********************************************************/
set_gpio:
    stmfd sp!, {r0,r1,lr}
    ldr r0, =GPIO1_SETDATAOUT
    ldr r1, =15<<21
    str r1, [r0]
    bl delay
    ldmfd sp!, {r0,r1,pc}

/********************************************************
  Clear gpio value
********************************************************/
clear_gpio:
	stmfd sp!, {r0,r1,lr}
    ldr r0, =GPIO1_CLEARDATAOUT
    ldr r1, =15<<21
    str r1, [r0]
    bl delay
    ldmfd sp!, {r0,r1,pc}

/********************************************************
  Registers (see TRM 20.4.4.1):
    WDT_BASE -> 0x44E35000
    WDT_WSPR -> 0x44E35048
    WDT_WWPS -> 0x44E35034
********************************************************/
gpio_setup:
    /* set clock for GPIO1, TRM 8.1.12.1.31 */
    ldr r0, =CM_PER_GPIO1_CLKCTRL
    ldr r1, =0x40002
    str r1, [r0]

    /* set pin 21-24 for output, led USR0, TRM 25.3.4.3 */
    ldr r0, =GPIO1_OE
    ldr r1, [r0]
    bic r1, r1, #(0xF<<21)
    str r1, [r0]
    bx lr

/********************************************************
  WDT disable sequence (see TRM 20.4.3.8):
    1- Write XXXX AAAAh in WDT_WSPR.
    2- Poll for posted write to complete using WDT_WWPS.W_PEND_WSPR. (bit 4)
    3- Write XXXX 5555h in WDT_WSPR.
    4- Poll for posted write to complete using WDT_WWPS.W_PEND_WSPR. (bit 4)
    
  Registers (see TRM 20.4.4.1):
    WDT_BASE -> 0x44E35000
    WDT_WSPR -> 0x44E35048
    WDT_WWPS -> 0x44E35034
********************************************************/
disable_wdt:
    stmfd sp!,{r0-r1,lr}
    ldr r0, =WDT_BASE
    
    ldr r1, =0xAAAA
    str r1, [r0, #0x48]
    bl poll_wdt_write

    ldr r1, =0x5555
    str r1, [r0, #0x48]
    bl poll_wdt_write

    ldmfd sp!,{r0-r1,pc}

poll_wdt_write:
    ldr r1, [r0, #0x34]
    and r1, r1, #(1<<4)
    cmp r1, #0
    bne poll_wdt_write
    bx lr
/********************************************************/

//Strings
data: .asciz "Data: "
hora: .asciz "Hora: "
minutos: .asciz "Minutos: "
segundos: .asciz "Segundos: "
quebra_linha: .asciz "\n\r"
