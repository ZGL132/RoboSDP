#include "apps/desktop-qt/third_party/qcustomplot/qcustomplot.h"

#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>

#include <algorithm>
#include <limits>

void QCPAxis::setLabel(const QString& label)
{
    m_label = label;
}

void QCPAxis::setRange(double lower, double upper)
{
    m_lower = lower;
    m_upper = upper;
}

const QString& QCPAxis::label() const
{
    return m_label;
}

double QCPAxis::lower() const
{
    return m_lower;
}

double QCPAxis::upper() const
{
    return m_upper;
}

void QCPLegend::setVisible(bool visible)
{
    m_visible = visible;
}

bool QCPLegend::visible() const
{
    return m_visible;
}

void QCPGraph::setName(const QString& name)
{
    m_name = name;
}

void QCPGraph::setPen(const QPen& pen)
{
    m_pen = pen;
}

void QCPGraph::setData(const QVector<double>& x, const QVector<double>& y)
{
    m_x = x;
    m_y = y;
}

const QString& QCPGraph::name() const
{
    return m_name;
}

const QPen& QCPGraph::pen() const
{
    return m_pen;
}

const QVector<double>& QCPGraph::xData() const
{
    return m_x;
}

const QVector<double>& QCPGraph::yData() const
{
    return m_y;
}

QCustomPlot::QCustomPlot(QWidget* parent)
    : QWidget(parent)
    , xAxis(new QCPAxis())
    , yAxis(new QCPAxis())
    , legend(new QCPLegend())
{
    setMinimumHeight(220);
    setAutoFillBackground(true);
    xAxis->setLabel(QStringLiteral("X"));
    yAxis->setLabel(QStringLiteral("Y"));
}

QCustomPlot::~QCustomPlot()
{
    for (QCPGraph* graph : m_graphs)
    {
        delete graph;
    }
    delete xAxis;
    delete yAxis;
    delete legend;
}

QCPGraph* QCustomPlot::addGraph()
{
    // 当前只需要最小多曲线兼容接口，因此按需追加到内部列表即可。
    auto* graph = new QCPGraph();
    m_graphs.push_back(graph);
    return graph;
}

QCPGraph* QCustomPlot::graph(int index)
{
    if (index < 0 || index >= static_cast<int>(m_graphs.size()))
    {
        return nullptr;
    }
    return m_graphs[static_cast<std::size_t>(index)];
}

void QCustomPlot::clearGraphs()
{
    for (QCPGraph* graph : m_graphs)
    {
        delete graph;
    }
    m_graphs.clear();
}

void QCustomPlot::rescaleAxes()
{
    if (m_graphs.empty())
    {
        xAxis->setRange(0.0, 1.0);
        yAxis->setRange(-1.0, 1.0);
        return;
    }

    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();

    for (const QCPGraph* graph : m_graphs)
    {
        const QVector<double>& x = graph->xData();
        const QVector<double>& y = graph->yData();
        const int count = std::min(x.size(), y.size());
        for (int i = 0; i < count; ++i)
        {
            minX = std::min(minX, x.at(i));
            maxX = std::max(maxX, x.at(i));
            minY = std::min(minY, y.at(i));
            maxY = std::max(maxY, y.at(i));
        }
    }

    if (minX == std::numeric_limits<double>::max())
    {
        xAxis->setRange(0.0, 1.0);
        yAxis->setRange(-1.0, 1.0);
        return;
    }

    if (qFuzzyCompare(minX, maxX))
    {
        minX -= 1.0;
        maxX += 1.0;
    }
    if (qFuzzyCompare(minY, maxY))
    {
        minY -= 1.0;
        maxY += 1.0;
    }

    const double xPadding = (maxX - minX) * 0.08;
    const double yPadding = (maxY - minY) * 0.12;
    xAxis->setRange(minX - xPadding, maxX + xPadding);
    yAxis->setRange(minY - yPadding, maxY + yPadding);
}

void QCustomPlot::replot()
{
    update();
}

void QCustomPlot::paintEvent(QPaintEvent* event)
{
    QWidget::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(QStringLiteral("#FFFFFF")));

    const QRectF plotRect = BuildPlotRect();
    DrawAxes(painter, plotRect);

    if (m_graphs.empty())
    {
        painter.setPen(QColor(QStringLiteral("#667085")));
        painter.drawText(plotRect, Qt::AlignCenter, QStringLiteral("暂无曲线数据"));
        return;
    }

    DrawGraphs(painter, plotRect);
    DrawLegend(painter, plotRect);
}

QRectF QCustomPlot::BuildPlotRect() const
{
    return QRectF(56.0, 18.0, width() - 84.0, height() - 72.0);
}

QPointF QCustomPlot::MapToPixel(double x, double y, const QRectF& plotRect) const
{
    const double xRange = std::max(1e-9, xAxis->upper() - xAxis->lower());
    const double yRange = std::max(1e-9, yAxis->upper() - yAxis->lower());
    const double px = plotRect.left() + ((x - xAxis->lower()) / xRange) * plotRect.width();
    const double py = plotRect.bottom() - ((y - yAxis->lower()) / yRange) * plotRect.height();
    return QPointF(px, py);
}

void QCustomPlot::DrawAxes(QPainter& painter, const QRectF& plotRect) const
{
    painter.setPen(QPen(QColor(QStringLiteral("#D0D5DD")), 1.0));
    painter.drawRect(plotRect);

    painter.setPen(QColor(QStringLiteral("#344054")));
    painter.drawText(QRectF(plotRect.left(), plotRect.bottom() + 8.0, plotRect.width(), 20.0),
        Qt::AlignCenter,
        xAxis->label());

    painter.save();
    painter.translate(18.0, plotRect.center().y());
    painter.rotate(-90.0);
    painter.drawText(QRectF(-plotRect.height() / 2.0, -20.0, plotRect.height(), 20.0), Qt::AlignCenter, yAxis->label());
    painter.restore();

    painter.drawText(
        QRectF(plotRect.left(), plotRect.bottom() + 28.0, plotRect.width(), 16.0),
        Qt::AlignLeft,
        QStringLiteral("%1").arg(xAxis->lower(), 0, 'f', 3));
    painter.drawText(
        QRectF(plotRect.left(), plotRect.bottom() + 28.0, plotRect.width(), 16.0),
        Qt::AlignRight,
        QStringLiteral("%1").arg(xAxis->upper(), 0, 'f', 3));

    painter.drawText(
        QRectF(0.0, plotRect.top() - 8.0, plotRect.left() - 8.0, 16.0),
        Qt::AlignRight,
        QStringLiteral("%1").arg(yAxis->upper(), 0, 'f', 3));
    painter.drawText(
        QRectF(0.0, plotRect.bottom() - 8.0, plotRect.left() - 8.0, 16.0),
        Qt::AlignRight,
        QStringLiteral("%1").arg(yAxis->lower(), 0, 'f', 3));
}

void QCustomPlot::DrawGraphs(QPainter& painter, const QRectF& plotRect) const
{
    for (const QCPGraph* graph : m_graphs)
    {
        const QVector<double>& x = graph->xData();
        const QVector<double>& y = graph->yData();
        const int count = std::min(x.size(), y.size());
        if (count < 2)
        {
            continue;
        }

        QPainterPath path;
        path.moveTo(MapToPixel(x.at(0), y.at(0), plotRect));
        for (int i = 1; i < count; ++i)
        {
            path.lineTo(MapToPixel(x.at(i), y.at(i), plotRect));
        }

        painter.setPen(graph->pen());
        painter.drawPath(path);
    }
}

void QCustomPlot::DrawLegend(QPainter& painter, const QRectF& plotRect) const
{
    if (!legend->visible() || m_graphs.empty())
    {
        return;
    }

    const QCPGraph* graph = m_graphs.front();
    const QRectF legendRect(plotRect.right() - 180.0, plotRect.top() + 8.0, 164.0, 28.0);
    painter.setPen(QPen(QColor(QStringLiteral("#D0D5DD")), 1.0));
    painter.setBrush(QColor(QStringLiteral("#FFFFFF")));
    painter.drawRoundedRect(legendRect, 4.0, 4.0);

    painter.setPen(graph->pen());
    painter.drawLine(
        QPointF(legendRect.left() + 10.0, legendRect.center().y()),
        QPointF(legendRect.left() + 38.0, legendRect.center().y()));

    painter.setPen(QColor(QStringLiteral("#344054")));
    painter.drawText(
        QRectF(legendRect.left() + 46.0, legendRect.top(), legendRect.width() - 52.0, legendRect.height()),
        Qt::AlignVCenter | Qt::AlignLeft,
        graph->name());
}
