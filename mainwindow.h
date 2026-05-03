#pragma once

#include "flightplanner.h"
#include "flightplotwidget.h"

#include <QMainWindow>
#include <QButtonGroup>
#include <QLabel>
#include <QLineEdit>
#include <QRadioButton>
#include <QPushButton>
#include <QGroupBox>
#include <QStatusBar>
#include <QSplitter>
#include <optional>

// ---------------------------------------------------------------------------
// MainWindow – Qt6 equivalent of the MATLAB GUIDE-generated planner004 GUI.
//
// Left panel:  all input controls (radio groups + text fields)
// Right panel: FlightPlotWidget
// ---------------------------------------------------------------------------
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void onPlotClicked();

private:
    // ---- Helper to build a labelled QLineEdit ----
    QLineEdit* makeLineEdit(const QString& placeholder = {});

    // ---- Build sub-panel groups ----
    QGroupBox* buildScienceTargetGroup();
    QGroupBox* buildAirportGroup();
    QGroupBox* buildAircraftGroup();
    QGroupBox* buildUnitsGroup();
    QGroupBox* buildFlightParamGroup();
    QGroupBox* buildOutputGroup();

    // ---- Validate & collect params; returns nullopt on error ----
    std::optional<FlightPlanParams> collectParams();

    // ---- Widgets ----
    FlightPlotWidget* m_plot = nullptr;

    // Science target
    QButtonGroup* m_bgSciTarget = nullptr;
    QLineEdit*    m_custTLat    = nullptr;
    QLineEdit*    m_custTLong   = nullptr;

    // Airport
    QButtonGroup* m_bgAirport  = nullptr;
    QLineEdit*    m_custAPLat  = nullptr;
    QLineEdit*    m_custAPLong = nullptr;

    // Aircraft
    QButtonGroup* m_bgAircraft  = nullptr;
    QLineEdit*    m_custCruise  = nullptr;
    QLineEdit*    m_custRange   = nullptr;

    // Units
    QButtonGroup* m_bgUnits = nullptr;

    // Flight parameters
    QLineEdit* m_numLines    = nullptr;
    QLineEdit* m_lineLength  = nullptr;
    QLineEdit* m_lineSpacing = nullptr;
    QLineEdit* m_lineHeading = nullptr;
    QLineEdit* m_bankAngle   = nullptr;

    // Output
    QLineEdit* m_fileName = nullptr;
};
