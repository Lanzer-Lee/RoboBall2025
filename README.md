# USART
The availabe usart channel of Robomaster board A are: `UART2`, `USART3`, `USART6`, `USART7`, `USART8`. 

Among them, `UART2` can be connected to the DuPont line. `USART3`, `USART6`, `USART7`, `USART8` can be connected to `11257W00-4P-S`.

The `USART3` line sequence is different from `USART6`, `USART7` and `USART8`.

The details are in table:

|Channel|RX|TX|
|-|-|-|
|UART2|PD6|PD5|
|USART3|PD9|PD8|
|USART6|PG9|PG14|
|USART8|PE0|PE1|
|USART7|PE7|PE8|