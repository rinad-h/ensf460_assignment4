void IOinit(void) {
    // Pin RA6 (PB1) is connected to RB7 (CN23) on the explorer board
    TRISBbits.TRISB7 = 1;        // Set RB7 (CN23) as input for PB1
    CNPU2bits.CN23PUE = 1;       // Enable pull-up resistor on CN23
    
    // Configure Change Notification for CN23
    CNEN2bits.CN23IE = 1;        // Enable CN interrupt for CN23
    (void)PORTB;                 // Read PORTB to clear mismatch
    
    // Configure CN Interrupt
    IPC4bits.CNIP = 6;           // Set CN interrupt priority (e.g., 6)
    IFS1bits.CNIF = 0;           // Clear CN interrupt flag
    IEC1bits.CNIE = 1;           // Enable CN interrupt
}

// ---- Display bar graph on one line ----
void display_bargraph(uint16_t adc_value) {
    char line[80];
    const int BAR_WIDTH = 32; // Width of the bar graph
    
    // Scale 10-bit ADC value (0-1023) to bar width
    int num_stars = (int)(((long)adc_value * BAR_WIDTH) / 1023); 
    
    // Clamp values to be safe
    if (num_stars < 0) num_stars = 0;
    if (num_stars > BAR_WIDTH) num_stars = BAR_WIDTH;

    int pos = 0;
    // \r returns to the start of the line without a new line
    line[pos++] = '\r'; 

    // Add prefix
    const char *prefix = "Mode 0: ";
    strcpy(&line[pos], prefix);
    pos += strlen(prefix);

    // Add stars
    for (int i = 0; i < num_stars && pos < sizeof(line) - 1; i++) {
        line[pos++] = '*';
    }
    // Add padding spaces
    for (int i = num_stars; i < BAR_WIDTH && pos < sizeof(line) - 1; i++) {
        line[pos++] = ' ';
    }

    // Add hex value, ensuring 3 digits (0x000 to 0x3FF)
    pos += snprintf(&line[pos], sizeof(line) - pos, " 0x%03X", adc_value);
    line[pos] = '\0'; // Null-terminate the string

    Disp2String(line); // Send the full line to UART
}


void IOCheck(void) {
    // Retains state between calls. Needs to be static.
    static uint8_t stream_started = 0; 
    uint16_t adc_value;
    
    // ---- Handle PB1 presses (State TOGGLE logic) ----
    if (pb1_event) {
        pb1_event = 0; // Consume the event immediately
        
        if (current_mode == MODE_0_BARGRAPH) {
            // Mode 0 -> Mode 1 (Always changes mode)
            current_mode = MODE_1_STREAM;
            mode_changed = 1;
        } 
        else { // current_mode == MODE_1_STREAM
            // Mode 1: Pressing PB1 toggles the streaming state
            if (stream_started == 0) {
                // Not streaming: Start streaming
                stream_started = 1;
                Disp2String("STREAMING_START\r\n"); // <--- Print START here
            } else {
                // Already streaming: Stop stream and return to Mode 0
                stream_started = 0; // Explicitly stop streaming
                current_mode = MODE_0_BARGRAPH;
                mode_changed = 1;
                Disp2String("STREAMING_STOP\r\n"); // Optional: Add a stop message
            }
        }
    }
