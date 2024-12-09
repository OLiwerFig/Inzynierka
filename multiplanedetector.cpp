// multiplane_detector.cpp
#include "multiplanedetector.h"
#include <queue>
#include <set>
#include <QDebug>
#include <cmath>

bool MultiPlaneDetector::isValidSensorData(const QList<QList<int>>& sensorData) {
    if (sensorData.size() != 8) return false;
    for (const auto& row : sensorData) {
        if (row.size() != 8) return false;
        for (int value : row) {
            if (value <= 0 || value > 4000) return false;
        }
    }
    return true;
}

std::vector<MultiPlaneDetector::PlaneSegment> MultiPlaneDetector::analyzeColumnTrend(
    const QList<QList<int>>& data, int col) {
    std::vector<PlaneSegment> segments;
    const int MIN_SEGMENT_SIZE = 4;
    const double TREND_THRESHOLD = 3.0;

    QVector<int> values;
    for (const auto& row : data) {
        if (col < row.size()) {
            values.append(row[col]);
        }
    }

    // Filtr medianowy do redukcji szumu
    QVector<int> filteredValues = values;
    const int filterWindow = 3;
    for (int i = 1; i < values.size() - 1; ++i) {
        QVector<int> window = {values[i - 1], values[i], values[i + 1]};
        std::sort(window.begin(), window.end());
        filteredValues[i] = window[1];
    }

    QVector<double> trends;
    const int trendWindowSize = 5;

    // Analiza trendu
    for (int i = 0; i <= filteredValues.size() - trendWindowSize; ++i) {
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        for (int j = 0; j < trendWindowSize; ++j) {
            sumX += j;
            sumY += filteredValues[i + j];
            sumXY += j * filteredValues[i + j];
            sumX2 += j * j;
        }

        double slope = (trendWindowSize * sumXY - sumX * sumY) /
                       (trendWindowSize * sumX2 - sumX * sumX);
        trends.append(slope);
    }

    // Wykrywanie segmentów na podstawie trendów
    int currentStart = 0;
    bool increasing = trends[0] > TREND_THRESHOLD;

    for (int i = 1; i < trends.size(); ++i) {
        bool isIncreasing = trends[i] > TREND_THRESHOLD;
        bool isDecreasing = trends[i] < -TREND_THRESHOLD;

        if (isIncreasing != increasing || isDecreasing) {
            if (i - currentStart >= MIN_SEGMENT_SIZE) {
                PlaneSegment segment;
                segment.startRow = currentStart;
                segment.endRow = i;
                segment.isIncreasing = increasing;
                segments.push_back(segment);
            }
            currentStart = i;
            increasing = isIncreasing;
        }
    }

    // Dodanie ostatniego segmentu
    if (filteredValues.size() - currentStart >= MIN_SEGMENT_SIZE) {
        PlaneSegment segment;
        segment.startRow = currentStart;
        segment.endRow = filteredValues.size() - 1;
        segment.isIncreasing = increasing;
        segments.push_back(segment);
    }

    return segments;
}




std::vector<MultiPlaneDetector::Plane> MultiPlaneDetector::detectMultiplePlanes(
    const QList<QList<int>>& sensorData) {
    std::vector<Plane> detectedPlanes;

    if (!isValidSensorData(sensorData)) {
        qDebug() << "Nieprawidłowe dane czujnika";
        return detectedPlanes;
    }

    // Analiza trendów w oryginalnych danych (wiersze)
    std::vector<std::vector<PlaneSegment>> rowSegments;
    for (int row = 0; row < sensorData.size(); ++row) {
        auto segments = analyzeRowData(sensorData, row);
        rowSegments.push_back(segments);
        qDebug() << "Wiersz" << row << "zawiera" << segments.size() << "segmentów";
    }

    // Obróć dane o 90 stopni (analiza w kolumnach)
    QList<QList<int>> rotatedData = Draw::rotate90Right(sensorData);

    // Analiza trendów w obróconych danych (kolumny)
    std::vector<std::vector<PlaneSegment>> colSegments;
    for (int col = 0; col < rotatedData.size(); ++col) {
        auto segments = analyzeColumnTrend(rotatedData, col);
        colSegments.push_back(segments);
        qDebug() << "Kolumna" << col << "zawiera" << segments.size() << "segmentów";
    }

    // Znajdź granice na podstawie zmian trendów w wierszach i kolumnach
    std::map<int, int> boundaryVotes;
    for (const auto& rowSegs : rowSegments) {
        for (const auto& seg : rowSegs) {
            boundaryVotes[seg.endRow]++;
        }
    }
    for (const auto& colSegs : colSegments) {
        for (const auto& seg : colSegs) {
            boundaryVotes[seg.endRow]++;
        }
    }

    // Wybierz silne granice
    std::vector<int> boundaries;
    boundaries.push_back(0);  // Początek
    const int MIN_VOTES = 2;  // Minimalna liczba głosów dla granicy
    for (const auto& [boundary, votes] : boundaryVotes) {
        if (votes >= MIN_VOTES) {
            boundaries.push_back(boundary);
            qDebug() << "Dodano granicę na pozycji" << boundary;
        }
    }
    if (boundaries.back() != rotatedData[0].size() - 1) {
        boundaries.push_back(rotatedData[0].size() - 1);
    }

    // Dopasuj płaszczyzny do segmentów
    for (size_t i = 0; i < boundaries.size() - 1; ++i) {
        int start = boundaries[i];
        int end = boundaries[i + 1];

        // Wyodrębnij dane segmentu
        QList<QList<int>> segmentData;
        for (int row = start; row <= end; ++row) {
            if (row < sensorData.size()) {
                segmentData.append(sensorData[row]);
            }
        }

        try {
            Draw::FitResult result = Draw::iterativePlaneFitting(segmentData, 3, 10, 2.5);

            Plane plane;
            plane.params = result.plane;
            plane.startRow = start;
            plane.endRow = end;
            plane.meanResidual = result.residuals.mean();

            double azimuth = std::atan2(plane.params.a, plane.params.c) * 180.0 / M_PI;
            plane.params.azimuth_angle = azimuth;
            plane.isLeftPlane = azimuth < 0;

            qDebug() << "Dopasowano płaszczyznę:"
                     << "Wiersze" << start << "-" << end
                     << "Azymut:" << azimuth
                     << "Średni błąd:" << plane.meanResidual;

            detectedPlanes.push_back(plane);
        } catch (const std::exception& e) {
            qDebug() << "Błąd dopasowania płaszczyzny:" << e.what();
        }
    }

    return detectedPlanes;
}



std::vector<MultiPlaneDetector::PlaneSegment> MultiPlaneDetector::analyzeRowData(
    const QList<QList<int>>& data, int row) {
    std::vector<PlaneSegment> segments;
    const int VALUE_THRESHOLD = 10;
    const int MIN_SEGMENT_SIZE = 2;

    if (row >= data.size() || data[row].isEmpty()) {
        return segments;
    }

    QVector<int> values = data[row].toVector();
    int currentStart = 0;
    bool currentTrendIncreasing = false;

    if (values.size() > 1) {
        currentTrendIncreasing = values[1] > values[0];
    }

    for (int i = 1; i < values.size() - 1; ++i) {
        int prevDiff = values[i] - values[i-1];
        int nextDiff = values[i+1] - values[i];

        bool isChangePoint = false;

        // Sprawdź nagłe zmiany wartości
        if (std::abs(nextDiff) > VALUE_THRESHOLD) {
            isChangePoint = true;
        }

        // Sprawdź zmianę trendu
        if ((prevDiff > 0 && nextDiff < 0) || (prevDiff < 0 && nextDiff > 0)) {
            isChangePoint = true;
        }

        if (isChangePoint && (i - currentStart >= MIN_SEGMENT_SIZE)) {
            PlaneSegment segment;
            segment.startRow = currentStart;
            segment.endRow = i;
            segment.isIncreasing = currentTrendIncreasing;
            segments.push_back(segment);

            qDebug() << "Znaleziono segment w wierszu" << row
                     << "od" << currentStart
                     << "do" << i;

            currentStart = i;
            currentTrendIncreasing = nextDiff > 0;
        }
    }

    if (values.size() - currentStart >= MIN_SEGMENT_SIZE) {
        PlaneSegment segment;
        segment.startRow = currentStart;
        segment.endRow = values.size() - 1;
        segment.isIncreasing = currentTrendIncreasing;
        segments.push_back(segment);
    }

    return segments;
}

void MultiPlaneDetector::findCorners(const QList<QList<int>>& data, std::vector<Plane>& planes) {
    for (auto& plane : planes) {
        for (int row = plane.startRow; row <= plane.endRow; ++row) {
            auto rowSegments = analyzeRowData(data, row);
            if (rowSegments.size() > 1) {
                qDebug() << "Found potential corner in row" << row;
                for (const auto& segment : rowSegments) {
                    if (std::abs(segment.endRow - segment.startRow) >= 3) {
                        plane.cornerRows.push_back(row);
                        plane.cornerCols.push_back(segment.endRow);
                        qDebug() << "Added corner at" << row << "," << segment.endRow;
                        break;
                    }
                }
            }
        }
    }
}




QString MultiPlaneDetector::generatePlanesDescription(const std::vector<MultiPlaneDetector::Plane>& planes) {
    QString description;
    for (size_t i = 0; i < planes.size(); i++) {
        const auto& plane = planes[i];
        double angle = plane.params.azimuth_angle;

        QString direction = std::abs(angle) < 10.0 ? "PRZÓD" :
                                (angle < 0 ? "LEWO" : "PRAWO");

        description += QString("Płaszczyzna %1:\n").arg(i + 1);
        description += QString("  Kierunek: %1 (kąt: %2°)\n")
                           .arg(direction)
                           .arg(angle, 0, 'f', 1);
        description += QString("  Wiersze: %1 - %2\n")
                           .arg(plane.startRow)
                           .arg(plane.endRow);
        description += QString("  Średnie odchylenie: %1mm\n")
                           .arg(plane.meanResidual, 0, 'f', 2);

        if (!plane.cornerRows.empty()) {
            //description += "  Narożniki:\n";
            //for (size_t j = 0; j < plane.cornerRows.size(); ++j) {
              //  description += QString("    - Wiersz: %1, Kolumna: %2\n")
              //                     .arg(plane.cornerRows[j])
              //                     .arg(plane.cornerCols[j]);
           // }
        }
    }
    return description;
}


QList<QList<int>> MultiPlaneDetector::extractPlaneData(const QList<QList<int>>& data, int startRow, int endRow) {
    QList<QList<int>> planeData;

    qDebug() << "Extracting data from" << startRow << "to" << endRow;
    qDebug() << "Source data dimensions:" << data.size() << "x" << (data.isEmpty() ? 0 : data[0].size());

    // Sprawdź zakres
    if (startRow < 0 || endRow >= data.size() || startRow > endRow) {
        qDebug() << "Invalid range for data extraction";
        return planeData;
    }

    // Bezpieczne kopiowanie danych
    try {
        for (int i = startRow; i <= endRow; ++i) {
            if (i < data.size()) {
                planeData.append(data[i]);
            }
        }
        qDebug() << "Extracted data dimensions:" << planeData.size() << "x"
                 << (planeData.isEmpty() ? 0 : planeData[0].size());
    } catch (const std::exception& e) {
        qDebug() << "Exception during data extraction:" << e.what();
        return QList<QList<int>>();
    }

    return planeData;
}

std::vector<int> MultiPlaneDetector::findPlaneBoundaries(const QList<QList<int>>& data) {
    std::vector<std::vector<PlaneSegment>> allSegments;
    const int MIN_SEGMENT_SIZE = 3; // Minimalny rozmiar segmentu

    // Analizuj trendy we wszystkich kolumnach
    for (int col = 0; col < data[0].size(); col++) {
        allSegments.push_back(analyzeColumnTrend(data, col));
    }

    // Znajdź punkty zmiany trendu
    std::map<int, int> boundaryVotes;
    for (const auto& columnSegments : allSegments) {
        for (const auto& segment : columnSegments) {
            if (segment.endRow - segment.startRow >= MIN_SEGMENT_SIZE) {
                boundaryVotes[segment.endRow]++;
            }
        }
    }

    std::vector<int> boundaries;
    int minVotesRequired = data[0].size() / 3; // Wymagaj zgody przynajmniej 1/3 kolumn

    for (const auto& vote : boundaryVotes) {
        if (vote.second >= minVotesRequired) {
            boundaries.push_back(vote.first);
            qDebug() << "Found boundary at row" << vote.first << "with" << vote.second << "votes";
        }
    }

    // Usuń zbyt bliskie granice
    std::vector<int> filteredBoundaries;
    const int MIN_BOUNDARY_DISTANCE = 2;

    if (!boundaries.empty()) {
        filteredBoundaries.push_back(boundaries[0]);
        for (size_t i = 1; i < boundaries.size(); ++i) {
            if (boundaries[i] - filteredBoundaries.back() >= MIN_BOUNDARY_DISTANCE) {
                filteredBoundaries.push_back(boundaries[i]);
            }
        }
    }

    std::sort(filteredBoundaries.begin(), filteredBoundaries.end());

    // Dodaj początek i koniec jeśli ich nie ma
    if (!filteredBoundaries.empty()) {
        if (filteredBoundaries.front() > 0) {
            filteredBoundaries.insert(filteredBoundaries.begin(), 0);
        }
        if (filteredBoundaries.back() < static_cast<int>(data.size() - 1)) {
            filteredBoundaries.push_back(static_cast<int>(data.size() - 1));
        }
    } else {
        filteredBoundaries = {0, static_cast<int>(data.size() - 1)};
    }

    return filteredBoundaries;
}
