/*
OPENGLのデータをシリアル通信で20文字をおくります．
*/
void sent2openGL(float yaw,float pitch,float roll){
  if(Serial.available() >= 0){
    if( Serial.read() =='n'){
        Serial.print(make_send_num(roll));
        Serial.print(",");
        Serial.print(make_send_num(pitch));
        Serial.print(",");
        Serial.print(make_send_num(yaw));  
    }
  }   
}

//change the form for serial commutation
String make_send_num(float a){
      if(a<-100){
        return String(a,1);
      }
      else if(a<-10){
        return String(a,2);
      }
      else if(a<0){
        return String(a,3);
      }
      else if(a<10){
        return String(a,4);
      }
      else if(a<100){
        return String(a,3);
      }
      else{
        return String(a,2);
      }
}
