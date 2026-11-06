#include <QGuiApplication>
#include <QImage>
#include <QPainter>
#include <QTimer>
#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <QWaitCondition>
#include <QMetaType>
#include <QDebug>
#include <QtMath>
#include <QFont>
#include <QPen>
#include <QColor>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <fstream>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <atomic>
#include <vector>
#include <algorithm>
#include <cstring>

#include "vehicle-data.h"

Q_DECLARE_METATYPE(VehicleData)

// ============================================================
// Display / gauge configuration
// ============================================================

static constexpr int   DISP_WIDTH   = 320;
static constexpr int   DISP_HEIGHT  = 240;

static constexpr int   DC_GPIO      = 536;
static constexpr int   RESET_GPIO   = 537;

static constexpr uint32_t SPI_SPEED_HZ = 24000000;

static constexpr int   SPEED_MAX    = 160;
static constexpr int   SPEED_TICK   = 2;
static constexpr int   SPEED_MAJOR  = 20;

static constexpr float GAUGE_START_DEG = -225.0f;
static constexpr float GAUGE_END_DEG   =   45.0f;

static constexpr const char *VEHICLE_SHM_NAME = "/vehicle_data";

// ============================================================
// Low level GPIO helpers (sysfs)
// ============================================================
//
// These do blocking file I/O. That is fine here because every
// call site lives on the dedicated DisplayWorker thread and
// never touches the render/UI thread.

namespace hw {

void gpioExport(int pin)
{
    std::ofstream f("/sys/class/gpio/export");
    f << pin;
}

void gpioDirection(int pin, const char *dir)
{
    std::ofstream f("/sys/class/gpio/gpio" + std::to_string(pin) + "/direction");
    f << dir;

    if (f.fail())
        qDebug() << "gpioDirection failed:" << pin;
}

void gpioWrite(int pin, int value)
{
    std::ofstream f("/sys/class/gpio/gpio" + std::to_string(pin) + "/value");
    f << value;

    if (f.fail())
        qDebug() << "gpioWrite failed:" << pin << value;
}

// ------------------------------------------------------------
// SPI transport
// ------------------------------------------------------------

int spiFd = -1;

bool spiOpen()
{
    spiFd = open("/dev/spidev0.0", O_RDWR);
    if (spiFd < 0)
    {
        qDebug() << "Cannot open SPI";
        return false;
    }

    uint8_t  mode  = SPI_MODE_0;
    uint8_t  bits  = 8;
    uint32_t speed = SPI_SPEED_HZ;

    ioctl(spiFd, SPI_IOC_WR_MODE, &mode);
    ioctl(spiFd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    gpioExport(DC_GPIO);
    gpioExport(RESET_GPIO);

    usleep(100000);

    gpioDirection(DC_GPIO, "out");
    gpioDirection(RESET_GPIO, "out");

    return true;
}

void spiWriteBytes(const uint8_t *data, size_t len)
{
    struct spi_ioc_transfer tr = {};
    tr.tx_buf        = reinterpret_cast<unsigned long>(data);
    tr.rx_buf        = 0;
    tr.len           = static_cast<uint32_t>(len);
    tr.speed_hz      = SPI_SPEED_HZ;
    tr.bits_per_word = 8;

    ioctl(spiFd, SPI_IOC_MESSAGE(1), &tr);
}

// ------------------------------------------------------------
// ILI9341 command layer
// ------------------------------------------------------------

void writeCommand(uint8_t cmd)
{
    gpioWrite(DC_GPIO, 0);
    spiWriteBytes(&cmd, 1);
}

void writeData(uint8_t data)
{
    gpioWrite(DC_GPIO, 1);
    spiWriteBytes(&data, 1);
}

void writeDataBuf(const uint8_t *buf, size_t len)
{
    gpioWrite(DC_GPIO, 1);
    spiWriteBytes(buf, len);
}

void ili9341Reset()
{
    gpioWrite(RESET_GPIO, 1);
    usleep(50000);
    gpioWrite(RESET_GPIO, 0);
    usleep(50000);
    gpioWrite(RESET_GPIO, 1);
    usleep(150000);
}

void ili9341Init()
{
    ili9341Reset();

    writeCommand(0x01);
    usleep(150000);

    writeCommand(0x28);

    writeCommand(0xCF); writeData(0x00); writeData(0x83); writeData(0x30);
    writeCommand(0xED); writeData(0x64); writeData(0x03); writeData(0x12); writeData(0x81);
    writeCommand(0xE8); writeData(0x85); writeData(0x01); writeData(0x79);
    writeCommand(0xCB); writeData(0x39); writeData(0x2C); writeData(0x00); writeData(0x34); writeData(0x02);
    writeCommand(0xF7); writeData(0x20);
    writeCommand(0xEA); writeData(0x00); writeData(0x00);
    writeCommand(0xC0); writeData(0x26);
    writeCommand(0xC1); writeData(0x11);
    writeCommand(0xC5); writeData(0x35); writeData(0x3E);
    writeCommand(0xC7); writeData(0xBE);

    uint8_t madctl = 0x48;
    if (const char *env = std::getenv("MADCTL_VAL"))
        madctl = static_cast<uint8_t>(strtol(env, nullptr, 0));

    qDebug() << "Using MADCTL =" << Qt::hex << madctl;

    writeCommand(0x36); writeData(madctl);
    writeCommand(0x3A); writeData(0x55);
    writeCommand(0xB1); writeData(0x00); writeData(0x1B);
    writeCommand(0xF2); writeData(0x08);
    writeCommand(0x26); writeData(0x01);
    writeCommand(0x11);

    usleep(150000);

    writeCommand(0x29);
}

void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    writeCommand(0x2A);
    uint8_t col[4] = {
        static_cast<uint8_t>(x0 >> 8), static_cast<uint8_t>(x0),
        static_cast<uint8_t>(x1 >> 8), static_cast<uint8_t>(x1)
    };
    writeDataBuf(col, 4);

    writeCommand(0x2B);
    uint8_t row[4] = {
        static_cast<uint8_t>(y0 >> 8), static_cast<uint8_t>(y0),
        static_cast<uint8_t>(y1 >> 8), static_cast<uint8_t>(y1)
    };
    writeDataBuf(row, 4);

    writeCommand(0x2C);
}

// Converts the QImage to RGB565 and blasts it over SPI.
// This is the slow, potentially blocking part of the pipeline -
// it must only ever run on the DisplayWorker thread.
void pushImage(const QImage &img)
{
    setAddrWindow(0, 0, DISP_WIDTH - 1, DISP_HEIGHT - 1);

    static std::vector<uint8_t> buf(DISP_WIDTH * DISP_HEIGHT * 2);

    size_t idx = 0;
    for (int y = 0; y < DISP_HEIGHT; ++y)
    {
        const QRgb *line = reinterpret_cast<const QRgb *>(img.constScanLine(y));

        for (int x = 0; x < DISP_WIDTH; ++x)
        {
            QRgb px = line[x];

            uint16_t r = (qRed(px)   >> 3) & 0x1F;
            uint16_t g = (qGreen(px) >> 2) & 0x3F;
            uint16_t b = (qBlue(px)  >> 3) & 0x1F;

            uint16_t rgb565 = (r << 11) | (g << 5) | b;

            buf[idx++] = rgb565 >> 8;
            buf[idx++] = rgb565 & 0xFF;
        }
    }

    gpioWrite(DC_GPIO, 1);

    constexpr size_t chunk = 4096;
    for (size_t off = 0; off < buf.size(); off += chunk)
    {
        size_t len = std::min(chunk, buf.size() - off);
        spiWriteBytes(buf.data() + off, len);
    }
}

} // namespace hw

// ============================================================
// Renderer - pure CPU-side drawing into a QImage.
// No I/O of any kind happens here, so it is safe and cheap to
// run on the main/GUI thread every frame tick.
// ============================================================

class Renderer
{
public:
    Renderer() : img(DISP_WIDTH, DISP_HEIGHT, QImage::Format_ARGB32) {}

    float speedToAngle(float speed) const
    {
        speed = qBound(0.0f, speed, static_cast<float>(SPEED_MAX));
        float t = speed / static_cast<float>(SPEED_MAX);
        return GAUGE_START_DEG + t * (GAUGE_END_DEG - GAUGE_START_DEG);
    }

    void drawText(QPainter &p, const QString &text, float x, float y, float w, float h,
                  int size, bool bold = false, QColor color = Qt::white,
                  Qt::Alignment align = Qt::AlignCenter)
    {
        QFont font = p.font();
        font.setPointSize(size);
        font.setBold(bold);

        p.setFont(font);
        p.setPen(color);
        p.drawText(QRectF(x, y, w, h), align, text);
    }

    // --------------------------------------------------------
    // Battery - flipped vertically, terminal on bottom, fill
    // grows top -> down toward the terminal as charge increases.
    // --------------------------------------------------------
    void drawBattery(QPainter &p, float percent)
    {
        percent = qBound(0.0f, percent, 100.0f);

        const int x = 8, y = 32;
        const int w = 35, h = 62;

        p.setPen(QPen(Qt::white, 2));
        p.setBrush(Qt::NoBrush);
        p.drawRoundedRect(QRectF(x, y, w, h), 3, 3);

        p.setPen(Qt::NoPen);
        p.setBrush(Qt::white);
        p.drawRect(QRectF(x + 11, y + h, 13, 5));

        const int padding = 4;
        const int innerX  = x + padding;
        const int innerY  = y + padding;
        const int innerW  = w - padding * 2;
        const int innerH  = h - padding * 2;
        const int fillHeight = static_cast<int>(innerH * percent / 100.0f);

        p.setPen(Qt::NoPen);
        p.setBrush(Qt::white);
        if (fillHeight > 0)
            p.drawRect(QRectF(innerX, innerY, innerW, fillHeight));

        drawText(p, QString("%1%").arg(static_cast<int>(percent)),
                  0, y + h + 13, 52, 18, 8, true, Qt::white);
    }

    // --------------------------------------------------------
    // Turn indicators
    // --------------------------------------------------------
    void drawIndicator(QPainter &p, bool active, bool left, float x, float y)
    {
        QColor c = active ? QColor(50, 255, 70) : QColor(30, 80, 35);

        p.setPen(Qt::NoPen);
        p.setBrush(c);

        QPolygonF arrow;
        if (left)
        {
            arrow << QPointF(x + 18, y)      << QPointF(x + 4,  y + 7)  << QPointF(x + 18, y + 14)
                  << QPointF(x + 18, y + 10) << QPointF(x + 28, y + 10) << QPointF(x + 28, y + 4)
                  << QPointF(x + 18, y + 4);
        }
        else
        {
            arrow << QPointF(x + 4, y)      << QPointF(x + 18, y + 7)  << QPointF(x + 4, y + 14)
                  << QPointF(x + 4, y + 10) << QPointF(x - 6,  y + 10) << QPointF(x - 6, y + 4)
                  << QPointF(x + 4, y + 4);
        }

        p.drawPolygon(arrow);
    }

    // --------------------------------------------------------
    // Speedometer
    // --------------------------------------------------------
    void drawSpeedGauge(QPainter &p, const VehicleData &d)
    {
        const int cx = 160, cy = 111, radius = 82;

        QPen outerPen(QColor(25, 80, 120), 6);
        outerPen.setCapStyle(Qt::FlatCap);
        p.setPen(outerPen);
        p.drawArc(cx - radius, cy - radius, radius * 2, radius * 2,
                  static_cast<int>(GAUGE_START_DEG * 16),
                  static_cast<int>((GAUGE_END_DEG - GAUGE_START_DEG) * 16));

        auto drawZone = [&](float from, float to, QColor color)
        {
            QPen pen(color, 6);
            pen.setCapStyle(Qt::FlatCap);
            p.setPen(pen);

            float a0 = speedToAngle(from);
            float a1 = speedToAngle(to);

            p.drawArc(cx - radius, cy - radius, radius * 2, radius * 2,
                      static_cast<int>(a0 * 16), static_cast<int>((a1 - a0) * 16));
        };

        drawZone(0,   100,       QColor(0, 180, 210));
        drawZone(100, 130,       QColor(235, 200, 30));
        drawZone(130, SPEED_MAX, QColor(230, 40, 40));

        for (int s = 0; s <= SPEED_MAX; s += SPEED_TICK)
        {
            float angleDeg = speedToAngle(s);
            float rad = qDegreesToRadians(angleDeg);
            bool major = (s % SPEED_MAJOR == 0);

            float outerRadius = major ? radius - 3  : radius - 2;
            float innerRadius = major ? radius - 14 : radius - 8;

            QPen tickPen(major ? QColor(235, 240, 245) : QColor(80, 170, 200), major ? 2 : 1);
            p.setPen(tickPen);

            float ox = cx + std::cos(rad) * outerRadius;
            float oy = cy - std::sin(rad) * outerRadius;
            float ix = cx + std::cos(rad) * innerRadius;
            float iy = cy - std::sin(rad) * innerRadius;

            p.drawLine(QPointF(ix, iy), QPointF(ox, oy));

            if (major)
            {
                float tx = cx + std::cos(rad) * (radius - 27);
                float ty = cy - std::sin(rad) * (radius - 27);
                drawText(p, QString::number(s), tx - 13, ty - 8, 26, 16, 7, true);
            }
        }

        float needleAngle = speedToAngle(d.speed);
        float needleRad = qDegreesToRadians(needleAngle);
        float nx = cx + std::cos(needleRad) * (radius - 18);
        float ny = cy - std::sin(needleRad) * (radius - 18);

        QPen needlePen(QColor(255, 70, 70), 4);
        needlePen.setCapStyle(Qt::RoundCap);
        p.setPen(needlePen);
        p.drawLine(QPointF(cx, cy), QPointF(nx, ny));

        p.setPen(QPen(Qt::black, 1));
        p.setBrush(QColor(230, 230, 230));
        p.drawEllipse(QPointF(cx, cy), 6, 6);

        drawText(p, QString::number(static_cast<int>(d.speed)), 85, 88, 150, 42, 29, true);
        drawText(p, "km/h", 130, 126, 60, 15, 8, false, QColor(180, 190, 200));
    }

    // --------------------------------------------------------
    // Full frame
    // --------------------------------------------------------
    void draw(const VehicleData &d)
    {
        img.fill(QColor(3, 7, 12));

        QPainter p(&img);
        p.setRenderHint(QPainter::Antialiasing, true);

        drawText(p, "10:24", 55, 4, 55, 18, 9, true);
        drawText(p, "28 C", 112, 4, 50, 18, 8, false, QColor(120, 210, 255));

        drawIndicator(p, d.leftIndicator, true, 8, 8);
        drawIndicator(p, d.rightIndicator, false, 292, 8);

        drawBattery(p, d.batteryPercent);
        drawSpeedGauge(p, d);

        drawText(p, "GEAR", 270, 35, 45, 10, 6, true, QColor(120, 180, 220));

        QString gearText = (d.gear == 0) ? "N" : QString::number(d.gear);
        drawText(p, gearText, 270, 43, 45, 24, 17, true, QColor(70, 255, 80));
        static const char *kDriveModes[] = { "ECO", "NORMAL", "SPORT" };
        const char *modeText = (d.driveMode >= 0 && d.driveMode <= 2) ? kDriveModes[d.driveMode] : "---";
        drawText(p, modeText, 270, 70, 45, 18, 8, true, QColor(70, 255, 100));

        drawText(p, "TRIP A", 4, 156, 76, 11, 6, true, QColor(110, 190, 230));
        drawText(p, QString::number(d.tripA, 'f', 1) + " km", 4, 166, 76, 16, 8, true);

        drawText(p, "TRIP B", 83, 156, 76, 11, 6, true, QColor(110, 190, 230));
        drawText(p, QString::number(d.tripB, 'f', 1) + " km", 83, 166, 76, 16, 8, true);

        drawText(p, "ODO", 162, 156, 76, 11, 6, true, QColor(110, 190, 230));
        drawText(p, QString::number(d.odometer, 'f', 0) + " km", 162, 166, 76, 16, 8, true);

        drawText(p, "AVG", 241, 156, 75, 11, 6, true, QColor(110, 190, 230));
        drawText(p, QString::number(d.averageSpeed, 'f', 0), 241, 166, 75, 16, 9, true);

        drawText(p, "RANGE", 4, 195, 76, 10, 6, true, QColor(110, 190, 230));
        drawText(p, QString::number(d.range, 'f', 0) + " km", 4, 205, 76, 18, 8, true, QColor(255, 220, 50));

        drawText(p, "VOLT", 83, 195, 76, 10, 6, true, QColor(110, 190, 230));
        drawText(p, QString::number(d.batteryVoltage, 'f', 1) + " V", 83, 205, 76, 18, 8, true, QColor(80, 230, 100));

        drawText(p, "TEMP", 162, 195, 76, 10, 6, true, QColor(110, 190, 230));
        drawText(p, QString::number(d.temperature, 'f', 0) + " C", 162, 205, 76, 18, 8, true,
                 d.temperature > 95 ? QColor(255, 60, 60) : Qt::white);

        drawText(p, "RPM", 241, 195, 75, 10, 6, true, QColor(110, 190, 230));
        drawText(p, QString::number(static_cast<int>(d.rpm)), 241, 205, 75, 18, 8, true);

        if (d.highBeam)
            drawText(p, "HIGH", 90, 16, 35, 12, 6, true, QColor(60, 160, 255));

        if (d.abs)
            drawText(p, "ABS", 210, 16, 30, 12, 6, true, QColor(255, 190, 40));

        if (d.bluetooth)
            drawText(p, "BT", 245, 16, 25, 12, 6, true, QColor(70, 180, 255));

        p.end();
    }

    QImage img;
};

// ============================================================
// VehicleDataReader
//
// Lives on its own QThread. Polls the shared-memory vehicle
// struct on a self-rescheduling QTimer (never a sleep()/blocking
// loop) and emits a copy of the data. Because it runs on its own
// thread with its own event loop, a stall here (e.g. a slow or
// missing shared-memory segment) never blocks rendering or SPI.
// ============================================================

class VehicleDataReader : public QObject
{
    Q_OBJECT

public:
    explicit VehicleDataReader(QObject *parent = nullptr) : QObject(parent) {}

    bool openSharedMemory()
    {
        shmFd = shm_open(VEHICLE_SHM_NAME, O_RDWR, 0666);
        if (shmFd < 0)
        {
            qDebug() << "Cannot open shared memory:" << VEHICLE_SHM_NAME;
            return false;
        }

        void *ptr = mmap(nullptr, sizeof(VehicleData), PROT_READ, MAP_SHARED, shmFd, 0);
        if (ptr == MAP_FAILED)
        {
            qDebug() << "mmap failed";
            close(shmFd);
            shmFd = -1;
            return false;
        }

        sharedData = static_cast<VehicleData *>(ptr);
        qDebug() << "Shared vehicle data connected";
        return true;
    }

public slots:
    // Called once the reader's thread event loop has started.
    void start(int intervalMs = 50)
    {
        pollIntervalMs = intervalMs;
        running = true;
        poll();
    }

    void stop()
    {
        running = false;
    }

private slots:
    void poll()
    {
        if (!running)
            return;

        if (sharedData != nullptr)
        {
            VehicleData snapshot{};
            std::memcpy(&snapshot, sharedData, sizeof(VehicleData));
            emit dataReady(snapshot);
        }

        // Re-arm instead of sleeping: keeps this thread's event
        // loop free and non-blocking between reads.
        QTimer::singleShot(pollIntervalMs, this, &VehicleDataReader::poll);
    }

signals:
    void dataReady(VehicleData data);

private:
    VehicleData *sharedData = nullptr;
    int shmFd = -1;
    int pollIntervalMs = 50;
    std::atomic<bool> running{false};
};

// ============================================================
// DisplayWorker
//
// Owns the SPI/ILI9341 hardware and runs on a dedicated QThread
// with its own run() loop (not the default Qt event loop), since
// SPI ioctl() calls can legitimately block for the duration of
// the transfer. It never runs on the render/UI thread.
//
// Frames are handed over via submitFrame(), which only takes a
// mutex briefly to store the latest frame and signal a condition
// variable - it never blocks the caller waiting on the SPI write
// itself. If frames arrive faster than they can be pushed, older
// frames are simply dropped in favour of the newest one, so the
// display never lags behind under load.
// ============================================================

class DisplayWorker : public QThread
{
    Q_OBJECT

public:
    bool initHardware()
    {
        if (!hw::spiOpen())
            return false;

        hw::ili9341Init();
        return true;
    }

public slots:
    void submitFrame(const QImage &frame)
    {
        QMutexLocker locker(&mutex);
        pendingFrame = frame;
        hasFrame = true;
        frameAvailable.wakeOne();
    }

    void requestStop()
    {
        QMutexLocker locker(&mutex);
        stopping = true;
        frameAvailable.wakeOne();
    }

protected:
    void run() override
    {
        while (true)
        {
            QImage frame;

            {
                QMutexLocker locker(&mutex);

                while (!hasFrame && !stopping)
                    frameAvailable.wait(&mutex); // thread sleeps here, nothing else is blocked

                if (stopping)
                    break;

                frame = pendingFrame;
                hasFrame = false;
            }

            hw::pushImage(frame);
        }
    }

private:
    QMutex        mutex;
    QWaitCondition frameAvailable;
    QImage        pendingFrame;
    bool          hasFrame = false;
    bool          stopping = false;
};

// ============================================================
// Main
// ============================================================

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    qRegisterMetaType<VehicleData>("VehicleData");

    // --------------------------------------------------------
    // Display / SPI thread
    // --------------------------------------------------------
    DisplayWorker displayWorker;
    if (!displayWorker.initHardware())
    {
        qDebug() << "SPI/display initialization failed";
        return 1;
    }
    displayWorker.start(); // launches run() on its own OS thread

    // --------------------------------------------------------
    // Vehicle data thread
    // --------------------------------------------------------
    auto *reader = new VehicleDataReader;
    if (!reader->openSharedMemory())
    {
        qDebug() << "Vehicle shared memory unavailable";
        qDebug() << "Starting display with zero data";
        // Not fatal: allows the dashboard to come up even if the
        // CAN application hasn't created the shared segment yet.
    }

    QThread readerThread;
    reader->moveToThread(&readerThread);

    QObject::connect(&readerThread, &QThread::started, reader, [reader]() { reader->start(50); });
    QObject::connect(&readerThread, &QThread::finished, reader, &QObject::deleteLater);

    readerThread.start();

    // --------------------------------------------------------
    // Latest known vehicle snapshot, updated asynchronously from
    // the reader thread via a queued signal - the lambda below
    // always runs on the main thread, so no manual locking of
    // "currentData" is needed even though dataReady() fires from
    // a different thread.
    // --------------------------------------------------------
    VehicleData currentData{};

    QObject::connect(reader, &VehicleDataReader::dataReady, &app,
                      [&currentData](VehicleData d) { currentData = d; },
                      Qt::QueuedConnection);

    // --------------------------------------------------------
    // Render loop (main/GUI thread)
    //
    // Pure CPU-side QPainter work, no I/O - safe to run on a
    // QTimer here. Each finished frame is handed to the display
    // thread with a non-blocking queued call.
    // --------------------------------------------------------
    Renderer renderer;

    QTimer renderTimer;
    QObject::connect(&renderTimer, &QTimer::timeout, [&]()
    {
        renderer.draw(currentData);

        QMetaObject::invokeMethod(&displayWorker, "submitFrame",
                                   Qt::QueuedConnection,
                                   Q_ARG(QImage, renderer.img));
    });

    renderTimer.start(50); // ~20 FPS

    int ret = app.exec();

    // --------------------------------------------------------
    // Shutdown
    // --------------------------------------------------------
    reader->stop();
    readerThread.quit();
    readerThread.wait();

    displayWorker.requestStop();
    displayWorker.wait();

    return ret;
}

#include "main.moc"