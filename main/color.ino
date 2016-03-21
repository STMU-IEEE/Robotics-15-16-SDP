victim_color getColor() {
    digitalWrite(COLOR_LED_PIN,HIGH);
    delay(700); //wait for at least 1 new reading to occur
    victim_color result = hue2color(readHue());
    digitalWrite(COLOR_LED_PIN,LOW);
    return result;
}

//Calculate hue (color) detected, returned as 8-bit value (not on 360 degree scale!)
//Hypothesis: hue is more immune to changes in lighting than e.g. simply using red/green values.
//As written, there is room for optimization and improved accuracy--test first.
uint8_t readHue() {
    uint16_t tcs_r, tcs_g, tcs_b, tcs_c;  //red, green, blue, clear
    tcs.getRawData(&tcs_r,&tcs_g,&tcs_b,&tcs_c);
    CRGB tcs_rgb; //object from FastLED
    //scale to 8-bit (only need relative precision for hue;
    // ignore saturation and value)
    Serial.println("Color sensor readings:");
    Serial.print("R:\t");
    Serial.println(tcs_r);
    Serial.print("G:\t");
    Serial.println(tcs_g);
    Serial.print("B:\t");
    Serial.println(tcs_b);
    while(max(max(tcs_r,tcs_g),tcs_b) > 255) {
        tcs_r >>= 1;
        tcs_g >>= 1;
        tcs_b >>= 1;
    }
    Serial.println("Color sensor (scaled to 8 bit):");
    Serial.print("R:\t");
    Serial.println(tcs_rgb.r = tcs_r);
    Serial.print("G:\t");
    Serial.println(tcs_rgb.g = tcs_g);
    Serial.print("B:\t");
    Serial.println(tcs_rgb.b = tcs_b);
    CHSV tcs_hsv = rgb2hsv_approximate(tcs_rgb); //convert CRGB object to CHSV
    Serial.print("Hue (8-bit):\t");
    Serial.println(tcs_hsv.hue);
    return tcs_hsv.hue;
}

victim_color hue2color(uint8_t hue){
    victim_color result;
    int8_t cmp_yellow = abs((int8_t)(hue-YELLOW_HUE));
    int8_t cmp_red = abs((int8_t)(hue-RED_HUE));
    //test if closer to yellow or red
    if (cmp_yellow < cmp_red){
        result = yellow;
        Serial.println("yellow");
    } else {
        result = red;
        Serial.println("red");
    }
    return result;
}

