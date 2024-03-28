/**
 * @example CompButton.ino
 * 
 * @par How to Use
 * This example shows that when the button component on the Nextion screen is released,
 * the text of this button will plus one every time.      
 *
 * @author  Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date    2015/7/10
 * @updated 2016/12/25 bring HMI up to v0.32 to avoid too old issues
 * @convert by Patrick Martin, no other changes made
 * @copyright 
 * Copyright (C) 2014-2015 ITEAD Intelligent Systems Co., Ltd. \n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include "Nextion.h"
HardwareSerial Serial1(PA10,PA9);

bool Aquecimento1 = false;

const int luz = PB3;

//Declarando os objetos Nextion
//Exemplo: [page id:0,component id:1, component name: "b0"]

//Controle manual (page id:1)
int ConMan = 1;
NexButton UP1     = NexButton(ConMan, 3, "b0");
NexButton DOWN1   = NexButton(ConMan, 8, "b1");

NexButton SWITCH1 = NexButton(ConMan,5,"b4");
NexText TEMP1     = NexText  (ConMan, 7, "t1");


//Registrar quais elementos da tela são interativos, passivos ao toque.
NexTouch *nex_listen_list[] = {
    &UP1, //UP1
    &DOWN1, //DOWN1
    &SWITCH1, //SWITCH1
    NULL
};

/*
 * Button component pop callback function. 
 * In this example,the button's text value will plus one every time when it is released. 
 */
void SWITCH1PopCallback(void *ptr){
  if (Aquecimento1 == false){
    digitalWrite(luz, HIGH);
    TEMP1.setText("On");
    SWITCH1.Set_background_image_pic(4);
    Aquecimento1 = true;
  }

  else{
    digitalWrite(luz, LOW);
    TEMP1.setText("Off");
    SWITCH1.Set_background_image_pic(3);
    Aquecimento1 = false;
  }
}

void setup(void){
    nexInit(); //Configura o baudrate de debug e comunicação com o Display Nextion

    /* Register the pop event callback function of the current button component. */
    SWITCH1.attachPop(SWITCH1PopCallback, &SWITCH1);
    pinMode(luz, OUTPUT);
    dbSerialPrintln("setup done"); 
}

void loop(void)
{   
    /*
     * When a pop or push event occured every time,
     * the corresponding component[right page id and component id] in touch event list will be asked.
     */
    nexLoop(nex_listen_list);
}











