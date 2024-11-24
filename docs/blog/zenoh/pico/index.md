---
tags:
    - zenoh
    - pico
---


# Demo: pub struct from arduino
- HW: ESP32
- SW: Arduino
- run zeno router from docker

```c
#include <Arduino.h>
#include <WiFi.h>
#include <zenoh-pico.h>

// WiFi-specific parameters
#define SSID "ssid"
#define PASS "pasword"

// Client mode values (comment/uncomment as needed)
#define MODE "client"
#define CONNECT ""  // If empty, it will scout
// Peer mode values (comment/uncomment as needed)
// #define MODE "peer"
// #define CONNECT "udp/224.0.0.225:7447#iface=en0"

#define KEYEXPR "demo/example/zenoh-pico-pub"

z_owned_session_t s;
z_owned_publisher_t pub;
static int idx = 0;

struct Vector3
{
    char x;
    char y;
    char z;
};

void setup() {
    // Initialize Serial for debug
    Serial.begin(115200);
    while (!Serial) {
        delay(1000);
    }

    // Set WiFi in STA mode and trigger attachment
    Serial.print("Connecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
    }
    Serial.println("OK");

    // Initialize Zenoh Session and other parameters
    z_owned_config_t config;
    z_config_default(&config);
    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, MODE);
    if (strcmp(CONNECT, "") != 0) {
        zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY, CONNECT);
    }

    // Open Zenoh session
    Serial.print("Opening Zenoh Session...");
    if (z_open(&s, z_config_move(&config), NULL) < 0) {
        Serial.println("Unable to open session!");
        while (1) {
            ;
        }
    }
    Serial.println("OK");

    // Start read and lease tasks for zenoh-pico
    if (zp_start_read_task(z_session_loan_mut(&s), NULL) < 0 || zp_start_lease_task(z_session_loan_mut(&s), NULL) < 0) {
        Serial.println("Unable to start read and lease tasks\n");
        z_session_drop(z_session_move(&s));
        while (1) {
            ;
        }
    }

    // Declare Zenoh publisher
    Serial.print("Declaring publisher for ");
    Serial.print(KEYEXPR);
    Serial.println("...");
    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str_unchecked(&ke, KEYEXPR);
    if (z_declare_publisher(z_session_loan(&s), &pub, z_view_keyexpr_loan(&ke), NULL) < 0) {
        Serial.println("Unable to declare publisher for key expression!");
        while (1) {
            ;
        }
    }
    Serial.println("OK");
    Serial.println("Zenoh setup finished!");

    delay(300);
}

void loop() {
    delay(1000);
    uint8_t buf[3];

    struct Vector3 v;
    v.x = 1;
    v.y = 2;
    v.z = 3;

    buf[0] = (uint8_t)v.x;
    buf[1] = (uint8_t)v.y;
    buf[2] = (uint8_t)v.z;
    // Create payload
    z_owned_bytes_t payload;
    // z_bytes_copy_from_str(&payload, buf);
    z_bytes_copy_from_buf(&payload, (const uint8_t*)buf, sizeof(buf));
    if (z_publisher_put(z_publisher_loan(&pub), z_bytes_move(&payload), NULL) < 0) {
        Serial.println("Error while publishing data");
    }
}
```

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
; framework = espidf
upload_port = /dev/ttyUSB0
monitor_port= /dev/ttyUSB0
monitor_speed = 115200
build_flags =
    -DZENOH_COMPILER_GCC
    -DZENOH_ARDUINO_ESP32
lib_deps = https://github.com/eclipse-zenoh/zenoh-pico
[platformio]
build_dir = /home/user/projects/pico_esp/.pio/build
```

```bash
docker run --init --net host eclipse/zenoh:1.0.2

```


```python
import zenoh


def main(conf: zenoh.Config, key: str):
    # initiate logging
    zenoh.init_log_from_env_or("error")

    print("Opening session...")
    with zenoh.open(conf) as session:

        print(f"Declaring Subscriber on '{key}'...")

        def listener(sample: zenoh.Sample):
            print(
                # f">> [Subscriber] Received {sample.kind} ('{sample.key_expr}': '{sample.payload.to_string()}')"
                sample.payload.to_bytes()
            )

        session.declare_subscriber(key, listener)

        print("Press CTRL-C to quit...")
        while True:
            time.sleep(1)

            
if __name__ == "__main__":
    main(zenoh.Config(), "demo/example/**")

```