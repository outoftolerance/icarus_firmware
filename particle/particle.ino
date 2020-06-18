#define AIRPLANE_MODE_ALTITUDE 304.8
#define LOCATION_REPORT_TTL 60

void setup() 
{
    Serial.begin(57600);
    Serial1.begin(57600);
}

void loop() 
{
    int current_altitude = 0;
    bool cellular_enabled = true;
    String serial_data;

    while(1)
    {
        //Check for new data on serial port
        if(Serial1.available())
        {
            serial_data = Serial1.readStringUntil('\n');
        }

        //Check current altitude
        if(current_altitude <= AIRPLANE_MODE_ALTITUDE)
        {
            if(!cellular_enabled)
            {
                Cellular.on();
                cellular_enabled = true;
            }
        }
        else
        {
            if(cellular_enabled)
            {
                Cellular.off();
                cellular_enabled = false;
            }
        }

        //Check if connected
        if(cellular_enabled && Cellular.ready() && Particle.connected() && serial_data.length() > 0)
        {
            //Publish location report
            Particle.publish("message", serial_data, LOCATION_REPORT_TTL, PRIVATE);
            serial_data = "";
        }
    }
}