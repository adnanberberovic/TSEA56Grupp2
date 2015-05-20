
#include <stdio.h>
#include <math.h>


int angle(int fram, int bak){
    
    int vinkel;
    double kvot;
    double delta;
    double mellan = 20; //Avståndet mellan sensorerna
    
    delta = fram - bak;
    kvot = delta/mellan;
    vinkel = atan(kvot)*180/3.14; //int på vinkel om man vill ha heltal på graderna.
    
    //kanske måste lägga till en IF-sats här om vi vill invertera vinkel beroende på vilken sida man mäter på.
    //typ If left -> vinkel = vinkel * (-1)
    return vinkel;
    
}

int mittpunkt(int vinkel, int fram_h, int fram_v, int bak_h, int bak_v){
    
    int hyp_h;
    int hyp_v; //hypotinusan från centrum
    int kat_h; //katet = lägden från centrum av roboten till vägg
    int kat_v;
    
    hyp_h = ((fram_h + bak_h) / 2) + 10; // + 10 är längden från kanten till centrum på roboten
    hyp_v = ((fram_v + bak_v) / 2) + 10;
    
    kat_h = hyp_h * cos(vinkel*3.14/180);//Dessa kan slås ihop, men pedagogiskt just nu.
    kat_v = hyp_v * cos(vinkel*3.14/180);
    
    return 2;
    //Vad vill man skicka tillbaka? plus på höger sida minus på vänster eller 0 = väldigt mycket åt vänster 100/255 = väldigt mycket åt höger, eller vill man bara mäta en sida i taget?
}


int main(int argc, const char * argv[]) {
    converter();
    
    int vinkel1;
    int vinkel2;
    
    int fram1 = 35; //Testdata
    int bak1 = 15;
    int fram2 =35;
    int bak2 = 55;
    
    vinkel1 = angle(fram1,bak1);
    vinkel2 = angle(fram2,bak2);
    
    int position = mittpunkt(vinkel1, fram1, fram2, bak1, bak2);
    
    
    
    return 0;
}