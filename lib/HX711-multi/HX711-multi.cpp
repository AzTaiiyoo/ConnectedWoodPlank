#include <Arduino.h>
#include <HX711-multi.h>
#include <vector>
#include <cmath>

HX711MULTI::HX711MULTI(int count, byte *dout, byte pd_sck, byte gain) {
	PD_SCK 	= pd_sck;
	DOUT 	= dout; //TODO - make the input of dout to the function a const, or otherwise copy the values for local storage
	COUNT   = count;

	debugEnabled = false;

	pinMode(PD_SCK, OUTPUT);
	for (int i=0; i<count; i++) {
		pinMode(DOUT[i], INPUT);
	}
	set_gain(gain);

	OFFSETS = (long *) malloc(COUNT*sizeof(long));

	for (int i=0; i<COUNT; ++i) {
		OFFSETS[i] = 0;
	}
}

HX711MULTI::~HX711MULTI() {
	free(OFFSETS);
}

bool HX711MULTI::is_ready() { 
	bool result = true;
	for (int i = 0; i<COUNT; ++i) {
		if (digitalRead(DOUT[i]) == HIGH) {
			result = false;
		}
	}
	return result;
}

void HX711MULTI::set_gain(byte gain) {

	switch (gain) {
		case 128:		// channel A, gain factor 128
			GAIN = 1;
			break;
		case 64:		// channel A, gain factor 64
			GAIN = 3;
			break;
		case 32:		// channel B, gain factor 32
			GAIN = 2;
			break;
	}

	digitalWrite(PD_SCK, LOW);
	read(); //a read is needed to get gain setting to come into effect. (for the next read)
}

byte HX711MULTI::get_count() {
	return COUNT;
}

//TODO: write a function / state variable that 'learns' (stores/tracks) the expected noise figure from the cells, and automatically selects a reasonable 'tolerance' for tare.
//		i.e. a 'best recently seen stability'. Keep it up-to-date automatically by updating it with every read. (reads will probably need to be time-aware)

bool HX711MULTI::tare(byte times, uint16_t tolerance) {
    // Array to hold the readings for each load cell
    long values[COUNT];
    std::vector<std::vector<long>> allValues(COUNT, std::vector<long>(times));

    // Read multiple samples
    for (int i = 0; i < times; ++i) {
        readRaw(values);
        for (int j = 0; j < COUNT; ++j) {
            allValues[j][i] = values[j];
        }
    }

    // Calculate min, max, and mean values
    std::vector<long> minValues(COUNT, LONG_MAX);
    std::vector<long> maxValues(COUNT, LONG_MIN);
    std::vector<long> meanValues(COUNT, 0);

    for (int i = 0; i < COUNT; ++i) {
        for (int j = 0; j < times; ++j) {
            if (allValues[i][j] < minValues[i]) {
                minValues[i] = allValues[i][j];
            }
            if (allValues[i][j] > maxValues[i]) {
                maxValues[i] = allValues[i][j];
            }
            meanValues[i] += allValues[i][j];
        }
        meanValues[i] /= times;
    }

    // Check if the fluctuation is within the tolerance
    for (int i = 0; i < COUNT; ++i) {
        if (tolerance != 0 && times > 1) {
            if (abs(maxValues[i] - minValues[i]) > tolerance) {
                // One of the cells fluctuated more than the allowed tolerance, reject tare attempt
                if (debugEnabled) {
                    Serial.print("Rejecting tare: (");
                    Serial.print(i);
                    Serial.print(") ");
                    Serial.println(abs(maxValues[i] - minValues[i]));
                }
                return false;
            }
        }
    }

    // Set the offsets to the mean values
    for (int i = 0; i < COUNT; ++i) {
        OFFSETS[i] = meanValues[i];
    }

    return true;
}


//reads from all cahnnels and sets the values into the passed long array pointer (which must have at least 'count' cells allocated)
//if you are only reading to toggle the line, and not to get values (such as in the case of setting gains) you can pass NULL.
void HX711MULTI::read(long *result) {
    
    readRaw(result);
    
    // Datasheet indicates the value is returned as a two's complement value, so 'stretch' the 24th bit to fit into 32 bits. 
	if (NULL!=result) {
		for (int j = 0; j < COUNT; ++j) {
		    result[j] -= OFFSETS[j];

		}
	}
}


void HX711MULTI::readRaw(long *result) {
	int i,j;
	// wait for all the chips to become ready
	while (!is_ready());

	// pulse the clock pin 24 times to read the data
	for (i = 0; i < 24; ++i) {
		digitalWrite(PD_SCK, HIGH);
		if (NULL!=result) {
			for (j = 0; j < COUNT; ++j) {
				bitWrite(result[j], 23-i, digitalRead(DOUT[j]));
			}
		}
		digitalWrite(PD_SCK, LOW);
	}
   
	// set the channel and the gain factor for the next reading using the clock pin
	for (i = 0; i < GAIN; ++i) {
		digitalWrite(PD_SCK, HIGH);
		digitalWrite(PD_SCK, LOW);
	}

    // Datasheet indicates the value is returned as a two's complement value, so 'stretch' the 24th bit to fit into 32 bits. 
    if (NULL!=result) {
	    for (j = 0; j < COUNT; ++j) {
	    	if ( ( result[j] & 0x00800000 ) ) {
	    		result[j] |= 0xFF000000;
	    	} else {
	    		result[j] &= 0x00FFFFFF; //required in lieu of re-setting the value to zero before shifting bits in.
	    	}
	    } 

    }
}

void HX711MULTI::setDebugEnable(bool debugEnable) {
	debugEnabled = debugEnable;
}

void HX711MULTI::power_down() {
	digitalWrite(PD_SCK, LOW);
	digitalWrite(PD_SCK, HIGH);
}

void HX711MULTI::power_up() {
	digitalWrite(PD_SCK, LOW);
}
