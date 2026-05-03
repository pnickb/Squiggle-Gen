#pragma once

#include "flightplanner.h"
#include <QWidget>

// ---------------------------------------------------------------------------
// FlightPlotWidget – custom widget that renders a FlightPlanResult using
// Qt6's QPainter.  Call setResult() to update the displayed flight path.
// ---------------------------------------------------------------------------
class FlightPlotWidget : public QWidget
{
    Q_OBJECT

public:
    explicit FlightPlotWidget(QWidget* parent = nullptr);

    // Replace the current plan with a new result and repaint.
    void setResult(const FlightPlanResult& result);

    // Clear the plot.
    void clear();

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    FlightPlanResult m_result;
    bool             m_hasData = false;

    // Computed during layout; maps data coordinates to widget pixels.
    struct Transform {
        double scaleX = 1.0;
        double scaleY = 1.0;
        double offsetX = 0.0;
        double offsetY = 0.0;
    };

    Transform computeTransform(int margin = 30) const;
    QPointF   toWidget(double x, double y, const Transform& t) const;
};
