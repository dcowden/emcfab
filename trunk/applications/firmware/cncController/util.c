//////////////////////////////////////////////////////////////////
//    Utility Functions
//
//
//////////////////////////////////////////////////////////////////
// --- Copying strings from ROM to RAM
void strConstCpy(char *dest, const char *source) {
    while(*source){
      *dest++ = *source++ ;
      *dest = 0 ;
    }
}

void USART_Send_String( char *data){
    char c;
    while( *data != 0u ){
       c = *data;
       USART_Write(c);
       data++;
    }
    USART_Write(13);
}

int pushChar ( char ch ){
    if ( ch == 10u || ch == 13u){
       //txtPos++;
       cmdBuffer[txtPos] = 0;
       if ( txtPos > 0u ){
         txtPos = 0;
         return 1u;
       }
       else{
         return 0u;
       }

    }
    else{
       Usart_write(ch);  //enable this line to echo input
       cmdBuffer[txtPos] = ch;
       txtPos++;
       return 0;
    }
}

void printMessage(const char* msg ){
  USART_Write(13);
  strConstCpy(txtBuffer,msg);
  USART_Send_String(txtBuffer);

}