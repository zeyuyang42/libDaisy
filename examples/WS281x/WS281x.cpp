/** Example of driving a WS2812B (NeoPixel) LED strip with the WS281xSpi driver
 *
 *  Connect the strip's DIN to D10 (SPI1 MOSI), and share GND with the Daisy.
 *  D8 (SPI1 SCK) is still driven by the peripheral but is left unconnected.
 *
 *  A single pixel chases along the strip, cycling red -> green -> blue on
 *  each lap. If the colors appear swapped, change the color order.
 */
#include "daisy_seed.h"

/** This prevents us from having to type "daisy::" in front of a lot of things. */
using namespace daisy;

/** Global Hardware access */
DaisySeed hw;

/** Number of LEDs in the strip */
static constexpr uint16_t kNumLeds = 8;

/** The SPI transmit buffer must live in DMA-accessible memory */
static uint8_t DMA_BUFFER_MEM_SECTION
    led_buffer[WS281xSpi::GetBufferSize(kNumLeds)];

/** Global LED strip object */
WS281xSpi leds;

int main(void)
{
    /** Initialize our hardware */
    hw.Init();

    /** Configure the strip: SPI1, data on D10, GRB wire order */
    WS281xSpi::Config cfg;
    cfg.Defaults();
    cfg.num_pixels = kNumLeds;
    if(leds.Init(cfg, led_buffer, sizeof(led_buffer)) != WS281xSpi::Result::OK)
    {
        /** Blink the onboard LED forever if initialization failed */
        while(1)
        {
            hw.SetLed(true);
            System::Delay(100);
            hw.SetLed(false);
            System::Delay(100);
        }
    }

    uint16_t current = 0;
    uint8_t  lap     = 0;

    /** Infinite Loop */
    while(1)
    {
        leds.Clear();
        leds.SetPixelColor(
            current, lap == 0 ? 64 : 0, lap == 1 ? 64 : 0, lap == 2 ? 64 : 0);
        /** Encode and start a non-blocking SPI DMA transfer */
        leds.ShowDma();

        System::Delay(100);

        if(++current >= kNumLeds)
        {
            current = 0;
            lap     = (lap + 1) % 3;
        }
    }
}
