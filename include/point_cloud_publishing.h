#ifndef COIN_LIO_POINT_CLOUD_PUBLISHING_H_
#define COIN_LIO_POINT_CLOUD_PUBLISHING_H_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "preprocess.h"

struct EIGEN_ALIGN16 PointXYZIReflectivity
{
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t reflectivity;
    float corrected_intensity;
    double time;
    std::uint32_t source_index;
    std::uint8_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIReflectivity,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, reflectivity, reflectivity)
    (float, corrected_intensity, corrected_intensity)
    (double, time, time)
    (std::uint32_t, source_index, source_index)
    (std::uint8_t, ring, ring)
)

// Coordinates may come from the scan before motion undistortion, while all
// attributes (including the image-processed intensity) come from the same point
// in the processed scan. UndistortPcl preserves point order.
inline PointXYZIReflectivity makePublishedPoint(const PointType &coordinates,
                                               const PointType &attributes,
                                               double scan_begin_time)
{
    PointXYZIReflectivity point{};
    point.x = coordinates.x;
    point.y = coordinates.y;
    point.z = coordinates.z;
    point.intensity = attributes.intensity;
    point.reflectivity = static_cast<std::uint16_t>(attributes.data_n[3]);
    point.corrected_intensity = attributes.normal_z;
    // curvature stores the offset from scan start in milliseconds; publish
    // absolute ROS acquisition time in seconds, independently of header.stamp.
    point.time = scan_begin_time + static_cast<double>(attributes.curvature) * 1e-3;
    point.source_index = static_cast<std::uint32_t>(attributes.normal_x);
    point.ring = static_cast<std::uint8_t>(attributes.data_c[2]);
    return point;
}

// Keep the existing PointXYZINormal fields and append the sensor's ring ID.
// Read ring from the source scan because coordinate transforms only copy XYZ
// and intensity into the published world/body clouds.
inline void addRingField(const PointCloudXYZI &source, sensor_msgs::PointCloud2 &message)
{
    sensor_msgs::PointField field;
    field.name = "ring";
    field.offset = message.point_step;
    field.datatype = sensor_msgs::PointField::UINT8;
    field.count = 1;
    message.fields.push_back(field);

    const std::uint32_t old_point_step = message.point_step;
    const std::uint32_t old_row_step = message.row_step;
    // Reserve a padded UINT8 slot so FLOAT32 fields stay aligned in each point.
    message.point_step += sizeof(float);
    message.row_step = message.width * message.point_step;
    std::vector<std::uint8_t> data(message.row_step * message.height);
    for (std::size_t row = 0; row < message.height; ++row)
    {
        for (std::size_t column = 0; column < message.width; ++column)
        {
            const std::size_t old_offset = row * old_row_step + column * old_point_step;
            const std::size_t offset = row * message.row_step + column * message.point_step;
            std::memcpy(data.data() + offset, message.data.data() + old_offset, old_point_step);
            data[offset + field.offset] =
                static_cast<std::uint8_t>(source.points[row * message.width + column].data_c[2]);
        }
    }
    message.data.swap(data);
}

#endif  // COIN_LIO_POINT_CLOUD_PUBLISHING_H_
