# c-atmega328p-rtc-i2c

**Project Description - English**

This project implements real-time clock (RTC) PCF8563 support on ATmega328P using the I2C bus. Additionally, it supports an alarm triggered by an external INT0 interrupt, allowing reaction to a set alarm time.

Features:

- Communication with PCF8563 via I2C interface

- RTC clock handling: setting and reading time and date

- Alarm setting with INT0 external interrupt handling

- Verification of I2C communication and device address

- Displaying the current time and date via USART

Hardware Requirements:

- ATmega328P microcontroller (e.g., Arduino Nano or standalone AVR)

- RTC PCF8563 connected to the I2C bus (SDA – PC4, SCL – PC5)

- USB-UART converter (e.g., CP2102, FT232RL) for monitoring data

- LED on PB5 as an alarm indicator

- Connecting wires

- 5V power supply

Usage Instructions:

- Compile the code in an AVR C-compatible environment (e.g., Atmel Studio, PlatformIO, AVR-GCC).

- Connect the PCF8563 RTC to ATmega328P (SDA – PC4, SCL – PC5, INT – PD2).

- Connect ATmega328P to a computer via a USB-UART converter (TX – RX, RX – TX, GND – GND).

- Set the initial time and alarm in the code.

- Run the program, open a serial terminal (e.g., PuTTY, Tera Term) at 9600 baud and monitor the current time and alarm triggering.

---------------------------------------------------------------------------------------------------------------------------------------------------------------

Opis projektu - Polski

Ten projekt implementuje obsługę zegara czasu rzeczywistego (RTC) PCF8563 na ATmega328P poprzez magistralę I2C. Dodatkowo obsługuje alarm wyzwalany przerwaniem zewnętrznym INT0, co umożliwia reagowanie na ustawiony czas alarmu.

Funkcjonalność:

- Komunikacja z PCF8563 przez interfejs I2C

- Obsługa zegara RTC: ustawianie i odczyt czasu oraz daty

- Ustawianie alarmu z obsługą przerwania INT0

- Weryfikacja komunikacji I2C oraz adresu urządzenia

- Wyświetlanie aktualnej godziny i daty przez USART

Wymagania sprzętowe:

- Mikrokontroler ATmega328P (np. Arduino Nano lub standalone AVR)

- RTC PCF8563 podłączony do magistrali I2C (SDA – PC4, SCL – PC5)

- Konwerter USB-UART (np. CP2102, FT232RL) do monitorowania danych

- Dioda LED na PB5 jako sygnalizacja alarmu

- Przewody połączeniowe

- Zasilanie 5V

Instrukcja użytkowania:

- Skompiluj kod w środowisku obsługującym AVR C (np. Atmel Studio, PlatformIO, AVR-GCC).

- Podłącz RTC PCF8563 do ATmega328P (SDA – PC4, SCL – PC5, INT – PD2).

- Podłącz ATmega328P do komputera przez konwerter USB-UART (TX – RX, RX – TX, GND – GND).

- Ustaw początkowy czas oraz alarm w kodzie.

- Uruchom program, otwórz terminal szeregowy (np. PuTTY, Tera Term) na 9600 baud i monitoruj aktualną godzinę i wyzwolenie alarmu.
