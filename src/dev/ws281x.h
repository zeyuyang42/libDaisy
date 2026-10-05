#pragma once
#ifndef DSY_WS281X_H
#define DSY_WS281X_H

#include <cstring>
#include "per/spi.h"
#include "util/color.h"

namespace daisy
{
/**
 * \brief SPI transport for WS281x LEDs (WS2812B and compatible).
 *
 * \details Uses ONE_LINE SPI (MOSI-only). The SPI clock is still generated
 *          on the SCK pin but is ignored by WS281x receivers. Only the MOSI
 *          (data) line connects to the LED strip.
 */
class WS281xSpiTransport
{
  public:
    struct Config
    {
        SpiHandle::Config::Peripheral periph; /**< SPI peripheral */
        SpiHandle::Config::BaudPrescaler
            baud_prescaler; /**< SPI baud rate prescaler */
        Pin sck_pin; /**< SCK pin (required by SPI peripheral, not connected to LEDs) */
        Pin data_pin; /**< MOSI data pin — connect to LED DIN */

        void Defaults()
        {
            periph         = SpiHandle::Config::Peripheral::SPI_1;
            baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_4;
            sck_pin        = Pin(PORTG, 11); // D8 on Daisy Seed
            data_pin       = Pin(PORTB, 5);  // D10 on Daisy Seed
        }
    };

    void Init(Config& config)
    {
        SpiHandle::Config spi_cfg;
        spi_cfg.periph          = config.periph;
        spi_cfg.mode            = SpiHandle::Config::Mode::MASTER;
        spi_cfg.direction       = SpiHandle::Config::Direction::ONE_LINE;
        spi_cfg.clock_polarity  = SpiHandle::Config::ClockPolarity::LOW;
        spi_cfg.clock_phase     = SpiHandle::Config::ClockPhase::ONE_EDGE;
        spi_cfg.datasize        = 8;
        spi_cfg.nss             = SpiHandle::Config::NSS::SOFT;
        spi_cfg.baud_prescaler  = config.baud_prescaler;
        spi_cfg.pin_config.sclk = config.sck_pin;
        spi_cfg.pin_config.mosi = config.data_pin;
        spi_cfg.pin_config.miso = Pin();
        spi_cfg.pin_config.nss  = Pin();
        spi_.Init(spi_cfg);
    }

    bool Write(uint8_t* data, size_t size)
    {
        return spi_.BlockingTransmit(data, size) == SpiHandle::Result::OK;
    }

    /**
     * \brief Request a non-blocking DMA transmit.
     * \param start_callback   Optional callback invoked when the transfer starts.
     * \param end_callback     Optional callback invoked when transfer completes.
     * \param callback_context Optional user context pointer passed to callbacks.
     */
    bool WriteDma(uint8_t*                            data,
                  size_t                              size,
                  SpiHandle::StartCallbackFunctionPtr start_callback = nullptr,
                  SpiHandle::EndCallbackFunctionPtr   end_callback   = nullptr,
                  void* callback_context                             = nullptr)
    {
        return spi_.DmaTransmit(
                   data, size, start_callback, end_callback, callback_context)
               == SpiHandle::Result::OK;
    }

  private:
    SpiHandle spi_;
};


/**
 * \brief Device support for WS281x addressable RGB LEDs (WS2812B and compatible).
 *
 * \details Drives WS281x LEDs using SPI bit-encoding: each WS281x data bit is
 *          represented as one SPI byte (0x80 for '0', 0xF8 for '1'). This
 *          exploits the SPI peripheral to generate the ~800 kHz WS281x waveform
 *          without bit-banging and supports DMA for non-blocking transmission.
 *
 *          SPI1-3 are clocked from PLL2P (25 MHz), so PS_4 gives a 6.25 MHz
 *          SPI clock:
 *          - 1 SPI bit = 160 ns → 1 SPI byte (one WS281x bit) = 1.28 µs
 *          - Bit '0' (0x80): 160 ns high, 1120 ns low
 *          - Bit '1' (0xF8): 800 ns high, 480 ns low
 *          - One frame takes (280 + 24 * num_pixels) * 1.28 µs
 *
 *          The 160 ns '0' high time is shorter than the WS2812B datasheet's
 *          T0H minimum, but is reliably read as '0' by the strips tested.
 *
 * \warning The caller must supply a DMA-safe transmit buffer, typically allocated
 *          in D2 SRAM using the \c DMA_BUFFER_MEM_SECTION attribute. Use
 *          \ref GetBufferSize to determine the required size before calling \ref Init.
 *
 * \note Tested with WS2812B at PS_4 baud prescaler on Daisy Seed.
 *
 * \author Zeyu Yang
 * \date 2025
 */
template <typename Transport>
class WS281x
{
  public:
    enum class Result
    {
        OK,
        ERR_INVALID_ARGUMENT, /**< Index out of range or num_pixels out of bounds */
        ERR_BUFFER_TOO_SMALL, /**< Supplied DMA buffer is smaller than required */
        ERR_TRANSPORT,        /**< SPI transmit failed */
        ERR_BUSY,             /**< A DMA transfer is still in flight */
    };

    struct Config
    {
        /**
         * \brief RGB channel wire order.
         * \details WS2812B uses GRB. Encode as two 2-bit fields per channel:
         *          bits[5:4] = R position, bits[3:2] = G position, bits[1:0] = B position.
         */
        enum ColorOrder : uint8_t
        {
            //      R          G          B
            RGB = ((0 << 4) | (1 << 2) | (2)),
            RBG = ((0 << 4) | (2 << 2) | (1)),
            GRB = ((1 << 4) | (0 << 2) | (2)),
            GBR = ((2 << 4) | (0 << 2) | (1)),
            BRG = ((1 << 4) | (2 << 2) | (0)),
            BGR = ((2 << 4) | (1 << 2) | (0)),
        };

        typename Transport::Config
                   transport_config; /**< Transport-specific configuration */
        ColorOrder color_order;      /**< Pixel wire order (WS2812B = GRB) */
        uint16_t   num_pixels;       /**< Number of LEDs in the chain */

        void Defaults()
        {
            transport_config.Defaults();
            color_order = ColorOrder::GRB;
            num_pixels  = 1;
        }
    };

    WS281x() {}
    ~WS281x() {}

    /**
     * \brief Calculate the required DMA-safe transmit buffer size in bytes.
     * \param num_pixels Number of LEDs in the chain.
     * \return Required buffer size in bytes.
     */
    static constexpr size_t GetBufferSize(uint16_t num_pixels)
    {
        return kLeadingZeros + static_cast<size_t>(num_pixels) * kBytesPerLed
               + kResetBytes;
    }

    /**
     * \brief Initialize the WS281x driver.
     *
     * \param config   Driver configuration (transport, color order, num_pixels).
     * \param dma_buf  Pointer to a DMA-safe transmit buffer (D2 SRAM recommended).
     *                 Must be at least \ref GetBufferSize(config.num_pixels) bytes.
     * \param buf_size Size of the supplied buffer in bytes.
     * \return \ref Result::OK on success, or an error code on failure.
     */
    Result Init(Config& config, uint8_t* dma_buf, size_t buf_size)
    {
        if(config.num_pixels == 0 || config.num_pixels > kMaxNumPixels)
            return Result::ERR_INVALID_ARGUMENT;
        if(dma_buf == nullptr || buf_size < GetBufferSize(config.num_pixels))
            return Result::ERR_BUFFER_TOO_SMALL;

        num_pixels_ = config.num_pixels;
        spi_buf_    = dma_buf;
        buf_size_   = buf_size;

        r_offset_ = (config.color_order >> 4) & 0x03;
        g_offset_ = (config.color_order >> 2) & 0x03;
        b_offset_ = config.color_order & 0x03;

        transport_.Init(config.transport_config);
        Clear();

        // Transmit zeros once to put the strip in a known (all-off) state
        std::memset(spi_buf_, 0, GetBufferSize(num_pixels_));
        transport_.Write(spi_buf_, GetBufferSize(num_pixels_));

        return Result::OK;
    }

    /**
     * \brief Set the color of a single pixel.
     *
     * \param idx Index of the pixel (0-based).
     * \param r   8-bit red value.
     * \param g   8-bit green value.
     * \param b   8-bit blue value.
     * \return \ref Result::OK, or \ref Result::ERR_INVALID_ARGUMENT if idx is out of range.
     */
    Result SetPixelColor(uint16_t idx, uint8_t r, uint8_t g, uint8_t b)
    {
        if(idx >= num_pixels_)
            return Result::ERR_INVALID_ARGUMENT;
        uint8_t* px   = &pixel_buf_[idx * 3];
        px[r_offset_] = r;
        px[g_offset_] = g;
        px[b_offset_] = b;
        return Result::OK;
    }

    /**
     * \brief Set the color of a single pixel.
     *
     * \param idx   Index of the pixel (0-based).
     * \param color \ref Color object.
     * \return \ref Result::OK, or \ref Result::ERR_INVALID_ARGUMENT if idx is out of range.
     */
    Result SetPixelColor(uint16_t idx, const Color& color)
    {
        return SetPixelColor(idx, color.Red8(), color.Green8(), color.Blue8());
    }

    /**
     * \brief Set the color of a single pixel.
     *
     * \param idx   Index of the pixel (0-based).
     * \param color 32-bit packed RGB color (MSB ignored, format 0x00RRGGBB).
     * \return \ref Result::OK, or \ref Result::ERR_INVALID_ARGUMENT if idx is out of range.
     */
    Result SetPixelColor(uint16_t idx, uint32_t color)
    {
        return SetPixelColor(
            idx, (color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF);
    }

    /**
     * \brief Get the current color of a pixel.
     *
     * \param idx Index of the pixel (0-based).
     * \return Packed RGB value (0x00RRGGBB), or 0 if idx is out of range.
     */
    uint32_t GetPixelColor(uint16_t idx) const
    {
        if(idx >= num_pixels_)
            return 0;
        const uint8_t* px = &pixel_buf_[idx * 3];
        return (static_cast<uint32_t>(px[r_offset_]) << 16)
               | (static_cast<uint32_t>(px[g_offset_]) << 8)
               | static_cast<uint32_t>(px[b_offset_]);
    }

    /**
     * \brief Fill all pixels with one color.
     *
     * \param r 8-bit red value.
     * \param g 8-bit green value.
     * \param b 8-bit blue value.
     */
    void Fill(uint8_t r, uint8_t g, uint8_t b)
    {
        for(uint16_t i = 0; i < num_pixels_; i++)
            SetPixelColor(i, r, g, b);
    }

    /**
     * \brief Fill all pixels with one color.
     * \param color \ref Color object.
     */
    void Fill(const Color& color)
    {
        for(uint16_t i = 0; i < num_pixels_; i++)
            SetPixelColor(i, color);
    }

    /**
     * \brief Fill all pixels with one color.
     * \param color 32-bit packed RGB color (MSB ignored, format 0x00RRGGBB).
     */
    void Fill(uint32_t color)
    {
        for(uint16_t i = 0; i < num_pixels_; i++)
            SetPixelColor(i, color);
    }

    /**
     * \brief Turn off all pixels.
     * \note Does not transmit to hardware. Call \ref Show or \ref ShowDma afterwards.
     */
    void Clear() { std::memset(pixel_buf_, 0, sizeof(pixel_buf_)); }

    /**
     * \brief Encode pixel data and transmit to LEDs (blocking).
     * \return \ref Result::OK on success, or \ref Result::ERR_BUSY if a
     *         \ref ShowDma transfer is still in flight.
     */
    Result Show()
    {
        if(!spi_buf_)
            return Result::ERR_INVALID_ARGUMENT;
        if(busy_)
            return Result::ERR_BUSY;
        Encode_();
        return transport_.Write(spi_buf_, GetBufferSize(num_pixels_))
                   ? Result::OK
                   : Result::ERR_TRANSPORT;
    }

    /**
     * \brief Encode pixel data and start a non-blocking DMA transmit to LEDs.
     *
     * \details The DMA buffer (supplied at \ref Init) is read by the DMA until the
     *          transfer completes. While a transfer is in flight, \ref ShowDma and
     *          \ref Show return \ref Result::ERR_BUSY without touching the buffer;
     *          poll \ref IsBusy or use \p end_callback to know when to call again.
     *
     * \note The SPI DMA streams are shared by all SPI peripherals. If another
     *       peripheral's DMA transfer is running, the transfer cannot start now
     *       and \ref ShowDma returns \ref Result::ERR_BUSY; call it again later.
     *       \p end_callback is only invoked for transfers that started.
     *
     * \param end_callback     Optional callback invoked when DMA transfer ends.
     * \param callback_context Optional user context pointer passed to the callback.
     * \return \ref Result::OK if the DMA transfer was started, or
     *         \ref Result::ERR_BUSY if a DMA transfer is still in flight.
     */
    Result ShowDma(SpiHandle::EndCallbackFunctionPtr end_callback     = nullptr,
                   void*                             callback_context = nullptr)
    {
        if(!spi_buf_)
            return Result::ERR_INVALID_ARGUMENT;
        if(busy_)
            return Result::ERR_BUSY;
        Encode_();
        user_end_callback_     = end_callback;
        user_callback_context_ = callback_context;
        started_               = false;
        if(!transport_.WriteDma(spi_buf_,
                                GetBufferSize(num_pixels_),
                                DmaStartCallback_,
                                DmaEndCallback_,
                                this))
            return Result::ERR_TRANSPORT;
        if(!started_)
        {
            // SpiHandle only accepted the request into its queue because another
            // peripheral holds the SPI DMA; don't report a transfer that may never
            // run, and don't forward its end callback if it does run later.
            user_end_callback_ = nullptr;
            return Result::ERR_BUSY;
        }
        return Result::OK;
    }

    /** \return true while a \ref ShowDma transfer is in flight. */
    bool IsBusy() const { return busy_; }

  private:
    /** SPI byte encoding for WS281x bit '0': short high (160 ns), long low (1120 ns). */
    static constexpr uint8_t kBitZero = 0x80;
    /** SPI byte encoding for WS281x bit '1': long high (800 ns), short low (480 ns). */
    static constexpr uint8_t kBitOne = 0xF8;

    /** Leading zero bytes sent before pixel data to absorb the SPI startup glitch
     *  that would otherwise cause LED 0 to briefly flash. */
    static constexpr int kLeadingZeros = 40;
    /** SPI bytes per LED: 24 WS281x bits × 1 byte/bit. */
    static constexpr int kBytesPerLed = 24;
    /** Trailing zero bytes holding the data line low for ~307 µs, so back-to-back
     *  frames latch: newer WS2812B parts need > 280 µs of reset (older ones > 50 µs). */
    static constexpr int kResetBytes = 240;

    static const uint16_t kMaxNumPixels = 256;

    Transport transport_;
    uint16_t  num_pixels_{0};
    /** Internal pixel color buffer in wire order (3 bytes per pixel). */
    uint8_t pixel_buf_[kMaxNumPixels * 3]{};
    /** External DMA-safe transmit buffer provided by the caller at Init. */
    uint8_t* spi_buf_{nullptr};
    size_t   buf_size_{0};
    uint8_t  r_offset_{0}, g_offset_{1}, b_offset_{2};
    /** Set when a DMA transfer starts reading spi_buf_; cleared from the DMA end IRQ. */
    volatile bool busy_{false};
    /** Set by the start callback, which SpiHandle only calls when the transfer
     *  actually starts (not when it is queued behind another peripheral). */
    volatile bool                     started_{false};
    SpiHandle::EndCallbackFunctionPtr user_end_callback_{nullptr};
    void*                             user_callback_context_{nullptr};

    static void DmaStartCallback_(void* context)
    {
        auto* self     = static_cast<WS281x*>(context);
        self->busy_    = true;
        self->started_ = true;
    }

    static void DmaEndCallback_(void* context, SpiHandle::Result result)
    {
        auto* self  = static_cast<WS281x*>(context);
        self->busy_ = false;
        if(self->user_end_callback_)
            self->user_end_callback_(self->user_callback_context_, result);
    }

    static constexpr uint8_t EncodeBit_(uint8_t bit)
    {
        return bit ? kBitOne : kBitZero;
    }

    /** Encode pixel_buf_ into the SPI wire format in spi_buf_. */
    void Encode_()
    {
        int idx = 0;

        for(int i = 0; i < kLeadingZeros; i++)
            spi_buf_[idx++] = 0x00;

        for(int led = 0; led < num_pixels_; led++)
        {
            for(int byte_i = 0; byte_i < 3; byte_i++)
            {
                uint8_t val = pixel_buf_[led * 3 + byte_i];
                for(int bit = 7; bit >= 0; bit--)
                    spi_buf_[idx++] = EncodeBit_((val >> bit) & 0x01);
            }
        }

        while(idx < static_cast<int>(GetBufferSize(num_pixels_)))
            spi_buf_[idx++] = 0x00;
    }
};

/** \brief Convenience alias for WS281x with the default SPI transport. */
using WS281xSpi = WS281x<WS281xSpiTransport>;

} // namespace daisy

#endif // DSY_WS281X_H
