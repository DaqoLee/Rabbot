#include "shell_port.h"
#include "usart.h"
#include "log.h"

Shell shell;
char buffer[512];
char dataRcvd;
short userShellWrite(char *data, unsigned short len)
{
   // serialTransmit(&debugSerial, (uint8_t *)&data, 1, 0xFF);
    HAL_UART_Transmit(&huart1 , (uint8_t *)data, len, 0xFF);
    return len;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)            
{
	if (huart == &huart1)     //�жϴ��������ǲ��Ǿ��1�ľ��
	{
    shellHandler(&shell, (char)dataRcvd);
    //printf("%c \n", dataRcvd);
		HAL_UART_Receive_IT(&huart1 ,  (uint8_t*)&dataRcvd , 1);

	}
}

void uartLogWrite(char *buffer, short len)
{
  //  HAL_UART_Transmit(&huart1, (uint8_t *)buffer, len, 0x100);
//  if (uartLog.shell)
  {
      shellWriteEndLine(&shell, buffer, len);
  }
}

Log uartLog = {
  .write = uartLogWrite,
  .active = 1,
  .level = LOG_DEBUG
};


/**
 * @brief �û�shell��ʼ��
 * 
 */
void userShellInit(void)
{
    HAL_UART_Receive_IT(&huart1 ,  (uint8_t*)&dataRcvd , 1);
    shell.write = userShellWrite;
    //shell.read = userShellRead;
    //shell.lock = userShellLock;
   // shell.unlock = userShellUnlock;
    shellInit(&shell, buffer, 512);

    logRegister(&uartLog, &shell);

}


CEVENT_EXPORT(EVENT_INIT_STAGE2, userShellInit);