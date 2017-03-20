FreeRTOS Example

This application demonstrates the use rosserial_tivac on FreeRTOS on TI's Launchpad.

***  ATTENTION ****

This is just a small demo of mixing rosserial + freertos + C++.

Take into attention that rosserial was not designing to be concurrency safe. 
Having node handle's spin in one task and other tasks publishing will certainly result in unexpected behavior and communication errors.
