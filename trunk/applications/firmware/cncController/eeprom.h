typedef unsigned char byte;
typedef unsigned int word;


void Eeprom_Write_Obj(word addr,void *obj,byte size);
void Eeprom_Read_Obj(word addr,void *obj,byte size);
