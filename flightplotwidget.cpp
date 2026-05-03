#include "flightplotwidget.h"

#include <QPainter>
#include <QPainterPath>
#include <QRectF>
#include <algorithm>
#include <cmath>

FlightPlotWidget::FlightPlotWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(400, 300);
    // White background
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::white);
    setPalette(pal);
}

void FlightPlotWidget::setResult(const FlightPlanResult& result)
{
    m_result  = result;
    m_hasData = result.valid && !result.xCoords.empty();
    update();
}

void FlightPlotWidget::clear()
{
    m_hasData = false;
    update();
}

// ---------------------------------------------------------------------------
// Compute the affine transform that maps data space to widget pixels.
// The y-axis is flipped so that increasing y goes upward (north).
// ---------------------------------------------------------------------------
FlightPlotWidget::Transform
FlightPlotWidget::computeTransform(int margin) const
{
    const auto& X = m_result.xCoords;
    const auto& Y = m_result.yCoords;

    double xMin = *std::min_element(X.begin(), X.end());
    double xMax = *std::max_element(X.begin(), X.end());
    double yMin = *std::min_element(Y.begin(), Y.end());
    double yMax = *std::max_element(Y.begin(), Y.end());

    // Also account for arc segments when computing bounds
    for (const auto& arc : m_result.arcs) {
        xMin = std::min(xMin, arc.cx - arc.r);
        xMax = std::max(xMax, arc.cx + arc.r);
        yMin = std::min(yMin, arc.cy - arc.r);
        yMax = std::max(yMax, arc.cy + arc.r);
    }

    double dataW = xMax - xMin;
    double dataH = yMax - yMin;
    if (dataW < 1e-12) dataW = 1.0;
    if (dataH < 1e-12) dataH = 1.0;

    double drawW = width()  - 2 * margin;
    double drawH = height() - 2 * margin;

    // Uniform scale (equal axes)
    double scale = std::min(drawW / dataW, drawH / dataH);

    Transform t;
    t.scaleX  =  scale;
    t.scaleY  = -scale; // flip y so north is up
    t.offsetX = margin + (drawW - dataW * scale) / 2.0 - xMin * scale;
    t.offsetY = margin + drawH - (drawH - dataH * scale) / 2.0 + yMin * scale;
    return t;
}

QPointF FlightPlotWidget::toWidget(double x, double y,
                                   const Transform& t) const
{
    return QPointF(t.offsetX + x * t.scaleX,
                   t.offsetY + y * t.scaleY);
}

// ---------------------------------------------------------------------------
// paintEvent
// ---------------------------------------------------------------------------
void FlightPlotWidget::paintEvent(QPaintEvent* /*event*/)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Background
    painter.fillRect(rect(), Qt::white);

    // Border
    painter.setPen(QPen(Qt::gray, 1));
    painter.drawRect(rect().adjusted(0, 0, -1, -1));

    if (!m_hasData) {
        painter.setPen(Qt::darkGray);
        painter.drawText(rect(), Qt::AlignCenter,
                         "No flight plan generated yet");
        return;
    }

    Transform t = computeTransform();

    // ---- Draw axis grid (light gray) ----
    {
        QPen gridPen(QColor(220, 220, 220), 1, Qt::DotLine);
        painter.setPen(gridPen);
        const int gridLines = 5;
        for (int i = 0; i <= gridLines; ++i) {
            int xp = static_cast<int>(30 + (width()  - 60) * i / gridLines);
            int yp = static_cast<int>(30 + (height() - 60) * i / gridLines);
            painter.drawLine(xp, 30, xp, height() - 30);
            painter.drawLine(30, yp, width() - 30, yp);
        }
    }

    // ---- Draw flight lines ----
    QPen linePen(QColor(30, 100, 200), 2);
    painter.setPen(linePen);

    const auto& X = m_result.xCoords;
    const auto& Y = m_result.yCoords;
    int n = m_result.lats.size();

    // Flight lines: pairs (0,1), (2,3), (4,5), ...
    for (int i = 0; i + 1 < n; i += 2) {
        QPointF p1 = toWidget(X[i],   Y[i],   t);
        QPointF p2 = toWidget(X[i+1], Y[i+1], t);
        painter.drawLine(p1, p2);
    }

    // ---- Draw turn arcs ----
    QPen arcPen(QColor(200, 80, 30), 2);
    painter.setPen(arcPen);
    painter.setBrush(Qt::NoBrush);

    for (const auto& arc : m_result.arcs) {
        // Build a polyline of 100 points along the arc (same as MATLAB DrawArc)
        constexpr int steps = 100;
        double a1r = arc.a1 * M_PI / 180.0;
        double a2r = arc.a2 * M_PI / 180.0;

        QPainterPath path;
        bool first = true;
        for (int k = 0; k < steps; ++k) {
            double theta = a1r + (a2r - a1r) * k / (steps - 1);
            double px = arc.cx + arc.r * std::cos(theta);
            double py = arc.cy + arc.r * std::sin(theta);
            QPointF wp = toWidget(px, py, t);
            if (first) { path.moveTo(wp); first = false; }
            else        { path.lineTo(wp); }
        }
        painter.drawPath(path);
    }

    // ---- Draw waypoint markers ----
    QPen ptPen(QColor(0, 160, 0), 1);
    painter.setPen(ptPen);
    painter.setBrush(QColor(0, 200, 0));
    for (int i = 0; i < n; ++i) {
        QPointF wp = toWidget(X[i], Y[i], t);
        painter.drawEllipse(wp, 4.0, 4.0);
    }

    // ---- Label start point ----
    painter.setPen(Qt::black);
    QFont f = painter.font();
    f.setPointSize(9);
    painter.setFont(f);
    if (!X.empty()) {
        QPointF wp = toWidget(X[0], Y[0], t);
        painter.drawText(wp + QPointF(6, -6), "Start");
    }
}

void FlightPlotWidget::resizeEvent(QResizeEvent* /*event*/)
{
    update();
}
