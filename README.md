# 16x2-LCD-Interface-with-STM32
Accept up to a 4 digit value from the virtual serial port on the STM Board. If the 4 digit value is one of the 10 pass-codes stored in an array, issue an auditory feedback signal and the text "access granted" to the serial port. If it is not one of the 10 allowable pass-codes, issue another appropriate auditory feedback signal and the text "access denied". Also illuminate a red led if the access is not granted and a green led if access is granted.
The software used for this embedded project is an IDE by STM also commonly reffered to as Cube MX.
Certain hardware are used for this project as well. Microcontroller - STM32L432KC. 16x2 LCD. 8-ohm speaker and LED.
