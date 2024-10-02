si#include <Pixy2.h>

// This is the main Pixy object 
Pixy2 pixy;

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  
  pixy.init();
  pinMode(4,OUTPUT);
}

void loop()
{ 
  int i; 
  // grab blocks!
  pixy.ccc.getBlocks();
  
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    for (i=0; i<pixy.ccc.numBlocks; i++)
    {
      Serial.print("  block ");
      Serial.print(i);
      Serial.print(": ");
      pixy.ccc.blocks[i].print();
    }

    Serial.println(pixy.ccc.blocks[0].m_signature);

    if(pixy.ccc.blocks[0].m_signature == 1)
    {
     digitalWrite(4,HIGH); 
     }

    if(pixy.ccc.blocks[0].m_signature == 3)
    {
     digitalWrite(4,HIGH);
     delay(500); 
     digitalWrite(4,LOW);
     delay(500);
     digitalWrite(4,HIGH);
     delay(500);
     digitalWrite(4,LOW);
     delay(500);
     }

     
}
else
  digitalWrite(4,LOW);  
}