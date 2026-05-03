#include "mainwindow.h"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QScrollArea>
#include <QSplitter>
#include <QVBoxLayout>
#include <QWidget>
#include <cmath>
#include <cstddef>

// Preset counts and Custom button IDs derived from the shared tables in
// flightplanner.h so that adding or removing a preset only requires a change
// in one place.
static constexpr int kSciPresetCount = static_cast<int>(std::size(kScienceTargets));
static constexpr int kSciCustomId    = kSciPresetCount;

static constexpr int kAPPresetCount  = static_cast<int>(std::size(kAirports));
static constexpr int kAPCustomId     = kAPPresetCount;

static constexpr int kAcPresetCount  = static_cast<int>(std::size(kAircraftPresets));
static constexpr int kAcCustomId     = kAcPresetCount;

static constexpr int kUnitPresetCount = static_cast<int>(std::size(kUnitConversions));

// ===========================================================================
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("CReSIS Flight Path Planner");
    setMinimumSize(950, 600);

    // ---- Central splitter ----
    auto* splitter = new QSplitter(Qt::Horizontal, this);
    setCentralWidget(splitter);

    // ---- Left scroll area ----
    auto* scrollArea = new QScrollArea;
    scrollArea->setWidgetResizable(true);
    scrollArea->setMinimumWidth(320);
    scrollArea->setMaximumWidth(400);

    auto* leftWidget = new QWidget;
    auto* leftLayout = new QVBoxLayout(leftWidget);
    leftLayout->setSpacing(6);
    leftLayout->setContentsMargins(8, 8, 8, 8);

    leftLayout->addWidget(buildScienceTargetGroup());
    leftLayout->addWidget(buildAirportGroup());
    leftLayout->addWidget(buildAircraftGroup());
    leftLayout->addWidget(buildUnitsGroup());
    leftLayout->addWidget(buildFlightParamGroup());
    leftLayout->addWidget(buildOutputGroup());

    auto* plotBtn = new QPushButton("Generate Flight Plan");
    plotBtn->setDefault(true);
    plotBtn->setMinimumHeight(36);
    QFont bf = plotBtn->font();
    bf.setBold(true);
    plotBtn->setFont(bf);
    connect(plotBtn, &QPushButton::clicked, this, &MainWindow::onPlotClicked);
    leftLayout->addWidget(plotBtn);

    leftLayout->addStretch();
    scrollArea->setWidget(leftWidget);
    splitter->addWidget(scrollArea);

    // ---- Right: plot widget ----
    m_plot = new FlightPlotWidget;
    splitter->addWidget(m_plot);

    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);

    // Status bar for distance / time output
    statusBar()->showMessage("Select parameters and press \"Generate Flight Plan\".");
}

// ---------------------------------------------------------------------------
QLineEdit* MainWindow::makeLineEdit(const QString& placeholder)
{
    auto* le = new QLineEdit;
    if (!placeholder.isEmpty())
        le->setPlaceholderText(placeholder);
    return le;
}

// ---------------------------------------------------------------------------
QGroupBox* MainWindow::buildScienceTargetGroup()
{
    auto* gb     = new QGroupBox("Science Target");
    auto* layout = new QVBoxLayout(gb);
    m_bgSciTarget = new QButtonGroup(this);

    for (int i = 0; i < kSciPresetCount; ++i) {
        auto* rb = new QRadioButton(kScienceTargets[i].name, gb);
        m_bgSciTarget->addButton(rb, i);
        layout->addWidget(rb);
    }
    auto* rbCustom = new QRadioButton("Custom", gb);
    m_bgSciTarget->addButton(rbCustom, kSciCustomId);
    layout->addWidget(rbCustom);

    // Custom lat/lon inputs (shown always; only used when Custom is selected)
    auto* row = new QHBoxLayout;
    m_custTLat  = makeLineEdit("Lat");
    m_custTLong = makeLineEdit("Lon");
    row->addWidget(new QLabel("Custom:"));
    row->addWidget(m_custTLat);
    row->addWidget(m_custTLong);
    layout->addLayout(row);

    // Select first preset by default
    m_bgSciTarget->button(0)->setChecked(true);
    return gb;
}

// ---------------------------------------------------------------------------
QGroupBox* MainWindow::buildAirportGroup()
{
    auto* gb     = new QGroupBox("Departure Airport");
    auto* layout = new QVBoxLayout(gb);
    m_bgAirport = new QButtonGroup(this);

    for (int i = 0; i < kAPPresetCount; ++i) {
        auto* rb = new QRadioButton(kAirports[i].name, gb);
        m_bgAirport->addButton(rb, i);
        layout->addWidget(rb);
    }
    auto* rbCustom = new QRadioButton("Custom", gb);
    m_bgAirport->addButton(rbCustom, kAPCustomId);
    layout->addWidget(rbCustom);

    auto* row = new QHBoxLayout;
    m_custAPLat  = makeLineEdit("Lat");
    m_custAPLong = makeLineEdit("Lon");
    row->addWidget(new QLabel("Custom:"));
    row->addWidget(m_custAPLat);
    row->addWidget(m_custAPLong);
    layout->addLayout(row);

    m_bgAirport->button(0)->setChecked(true);
    return gb;
}

// ---------------------------------------------------------------------------
QGroupBox* MainWindow::buildAircraftGroup()
{
    auto* gb     = new QGroupBox("Aircraft");
    auto* layout = new QVBoxLayout(gb);
    m_bgAircraft = new QButtonGroup(this);

    for (int i = 0; i < kAcPresetCount; ++i) {
        auto* rb = new QRadioButton(kAircraftPresets[i].name, gb);
        m_bgAircraft->addButton(rb, i);
        layout->addWidget(rb);
    }
    auto* rbCustom = new QRadioButton("Custom", gb);
    m_bgAircraft->addButton(rbCustom, kAcCustomId);
    layout->addWidget(rbCustom);

    // Custom cruise / range (in user units – interpreted after unit selection)
    auto* row = new QHBoxLayout;
    m_custCruise = makeLineEdit("Cruise speed");
    m_custRange  = makeLineEdit("Range");
    row->addWidget(new QLabel("Cruise:"));
    row->addWidget(m_custCruise);
    row->addWidget(new QLabel("Range:"));
    row->addWidget(m_custRange);
    layout->addLayout(row);

    m_bgAircraft->button(0)->setChecked(true);
    return gb;
}

// ---------------------------------------------------------------------------
QGroupBox* MainWindow::buildUnitsGroup()
{
    auto* gb     = new QGroupBox("Units");
    auto* layout = new QVBoxLayout(gb);
    m_bgUnits = new QButtonGroup(this);

    for (int i = 0; i < kUnitPresetCount; ++i) {
        auto* rb = new QRadioButton(kUnitConversions[i].name, gb);
        m_bgUnits->addButton(rb, i);
        layout->addWidget(rb);
    }

    // Default: metres
    m_bgUnits->button(0)->setChecked(true);
    return gb;
}

// ---------------------------------------------------------------------------
QGroupBox* MainWindow::buildFlightParamGroup()
{
    auto* gb     = new QGroupBox("Flight Parameters");
    auto* layout = new QFormLayout(gb);

    m_numLines    = makeLineEdit("e.g. 5");
    m_lineLength  = makeLineEdit("in selected units");
    m_lineSpacing = makeLineEdit("in selected units");
    m_lineHeading = makeLineEdit("0–360°");
    m_bankAngle   = makeLineEdit("default 15°");

    layout->addRow("Number of lines:",  m_numLines);
    layout->addRow("Line length:",      m_lineLength);
    layout->addRow("Line spacing:",     m_lineSpacing);
    layout->addRow("Line heading (°):", m_lineHeading);
    layout->addRow("Bank angle (°):",   m_bankAngle);

    return gb;
}

// ---------------------------------------------------------------------------
QGroupBox* MainWindow::buildOutputGroup()
{
    auto* gb     = new QGroupBox("Output File (optional)");
    auto* layout = new QFormLayout(gb);

    m_fileName = makeLineEdit("filename (no extension)");
    layout->addRow("GPS waypoint file:", m_fileName);

    return gb;
}

// ---------------------------------------------------------------------------
// collectParams – read all widget values and validate.
// Returns nullopt and shows an error message if something is missing/invalid.
// ---------------------------------------------------------------------------
std::optional<FlightPlanParams> MainWindow::collectParams()
{
    FlightPlanParams p;
    QStringList errors;

    // ---- Science target ----
    int sciId = m_bgSciTarget->checkedId();
    if (sciId < 0) {
        errors << "No science target selected.";
    } else if (sciId < kSciCustomId) {
        p.sciTarLat  = kScienceTargets[sciId].lat;
        p.sciTarLong = kScienceTargets[sciId].lon;
    } else {
        bool ok1, ok2;
        p.sciTarLat  = m_custTLat->text().toDouble(&ok1);
        p.sciTarLong = m_custTLong->text().toDouble(&ok2);
        if (!ok1 || !ok2)
            errors << "Custom science target lat/lon must be numeric.";
    }

    // ---- Airport (stored but not yet used in path calculation) ----
    int apId = m_bgAirport->checkedId();
    if (apId < 0) {
        errors << "No airport selected.";
    } else if (apId < kAPCustomId) {
        p.apLat  = kAirports[apId].lat;
        p.apLong = kAirports[apId].lon;
    } else {
        bool ok1, ok2;
        p.apLat  = m_custAPLat->text().toDouble(&ok1);
        p.apLong = m_custAPLong->text().toDouble(&ok2);
        if (!ok1 || !ok2)
            errors << "Custom airport lat/lon must be numeric.";
    }

    // ---- Units ----
    int unitId = m_bgUnits->checkedId();
    if (unitId < 0) {
        errors << "No unit system selected.";
    } else {
        p.speedConversion = kUnitConversions[unitId].speedConv;
        p.rangeConversion = kUnitConversions[unitId].rangeConv;
    }

    // ---- Aircraft ----
    int acId = m_bgAircraft->checkedId();
    if (acId < 0) {
        errors << "No aircraft selected.";
    } else if (acId < kAcCustomId) {
        p.cruise = kAircraftPresets[acId].cruise;
        p.range  = kAircraftPresets[acId].range;
    } else {
        bool ok1, ok2;
        double custCruise = m_custCruise->text().toDouble(&ok1);
        double custRange  = m_custRange->text().toDouble(&ok2);
        if (!ok1 || !ok2) {
            errors << "Custom cruise speed and range must be numeric.";
        } else {
            p.cruise = custCruise * p.speedConversion;
            p.range  = custRange  * p.rangeConversion;
        }
    }

    // ---- Flight parameters ----
    {
        bool ok;
        p.numLines = m_numLines->text().toInt(&ok);
        if (!ok || p.numLines < 1)
            errors << "Number of lines must be a positive integer.";
    }
    {
        bool ok;
        p.lineLength = m_lineLength->text().toDouble(&ok);
        if (!ok || p.lineLength <= 0.0)
            errors << "Line length must be a positive number.";
    }
    {
        bool ok;
        p.lineSpace = m_lineSpacing->text().toDouble(&ok);
        if (!ok || p.lineSpace <= 0.0)
            errors << "Line spacing must be a positive number.";
    }
    {
        bool ok;
        double h = m_lineHeading->text().toDouble(&ok);
        if (!ok)
            errors << "Line heading must be numeric.";
        else
            p.lineHeading = std::fmod(h, 360.0);
    }
    {
        // Bank angle is optional; default 15°
        if (!m_bankAngle->text().trimmed().isEmpty()) {
            bool ok;
            double ba = m_bankAngle->text().toDouble(&ok);
            if (!ok || ba <= 0.0 || ba >= 90.0)
                errors << "Bank angle must be between 0 and 90 degrees.";
            else
                p.bankAngle = ba;
        }
        // else keep default of 15°
    }

    // ---- Output file ----
    p.fileName = m_fileName->text().trimmed().toStdString();

    if (!errors.isEmpty()) {
        QMessageBox::warning(this, "Input Error",
                             errors.join("\n"));
        return std::nullopt;
    }
    return p;
}

// ---------------------------------------------------------------------------
void MainWindow::onPlotClicked()
{
    auto params = collectParams();
    if (!params) return;

    FlightPlanResult result = generateFlightPlan(*params);

    if (!result.valid) {
        QMessageBox::critical(this, "Error", QString::fromStdString(result.error));
        return;
    }

    m_plot->setResult(result);

    // Show summary in status bar
    QString msg = QString("Total distance: %1 | Flight time: %2 h | "
                          "Turn radius: %3 | Waypoints: %4")
        .arg(result.totalDistance, 0, 'f', 2)
        .arg(result.flightTime,    0, 'f', 2)
        .arg(result.turnRadius,    0, 'f', 2)
        .arg(static_cast<int>(result.lats.size()));
    statusBar()->showMessage(msg);

    if (!params->fileName.empty()) {
        statusBar()->showMessage(
            msg + "  |  Waypoints written to " +
            QString::fromStdString(params->fileName) + ".txt");
    }
}
