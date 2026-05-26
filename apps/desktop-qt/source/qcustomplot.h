#pragma once

#include <QColor>
#include <QPen>
#include <QVector>
#include <QWidget>

#include <vector>

/**
 * @brief 最小 QCustomPlot 兼容子集。
 *
 * 本文件只实现当前 RoboSDP 一阶段需要的单图单曲线展示能力，
 * 目的是在不引入复杂图表系统的前提下，提供与 QCustomPlot 近似的最小接线接口。
 */

class QCPAxis
{
public:
    /// 设置坐标轴标签文本。
    void setLabel(const QString& label);

    /// 设置坐标轴显示范围。
    void setRange(double lower, double upper);

    /// 读取坐标轴标签。
    const QString& label() const;

    /// 读取坐标轴范围下界。
    double lower() const;

    /// 读取坐标轴范围上界。
    double upper() const;

private:
    QString m_label;
    double m_lower = 0.0;
    double m_upper = 1.0;
};

class QCPLegend
{
public:
    /// 设置图例是否可见。
    void setVisible(bool visible);

    /// 返回图例可见状态。
    bool visible() const;

private:
    bool m_visible = false;
};

class QCPGraph
{
public:
    /// 设置曲线名称，用于图例或状态显示。
    void setName(const QString& name);

    /// 设置曲线画笔。
    void setPen(const QPen& pen);

    /// 写入曲线数据。
    void setData(const QVector<double>& x, const QVector<double>& y);

    /// 读取曲线名称。
    const QString& name() const;

    /// 读取曲线画笔。
    const QPen& pen() const;

    /// 读取 X 轴数据。
    const QVector<double>& xData() const;

    /// 读取 Y 轴数据。
    const QVector<double>& yData() const;

private:
    QString m_name;
    QPen m_pen = QPen(QColor(QStringLiteral("#1F6FEB")), 2.0);
    QVector<double> m_x;
    QVector<double> m_y;
};

class QCustomPlot : public QWidget
{
public:
    explicit QCustomPlot(QWidget* parent = nullptr);
    ~QCustomPlot() override;

    /// 新增一条曲线并返回其指针。
    QCPGraph* addGraph();

    /// 按索引读取曲线对象。
    QCPGraph* graph(int index);

    /// 清空当前图表内的所有曲线。
    void clearGraphs();

    /// 根据当前曲线数据自动重设坐标范围。
    void rescaleAxes();

    /// 触发一次重绘刷新。
    void replot();

protected:
    /// 绘制最小二维曲线视图。
    void paintEvent(QPaintEvent* event) override;

private:
    QRectF BuildPlotRect() const;
    QPointF MapToPixel(double x, double y, const QRectF& plotRect) const;
    void DrawAxes(QPainter& painter, const QRectF& plotRect) const;
    void DrawGraphs(QPainter& painter, const QRectF& plotRect) const;
    void DrawLegend(QPainter& painter, const QRectF& plotRect) const;

public:
    QCPAxis* xAxis = nullptr;
    QCPAxis* yAxis = nullptr;
    QCPLegend* legend = nullptr;

private:
    std::vector<QCPGraph*> m_graphs;
};
