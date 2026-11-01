#include <gtest/gtest.h>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "point_cloud_publishing.h"

TEST(PointCloudPublishing, PreprocessingPreservesAttributesAfterFiltering)
{
    pcl::PointCloud<ouster_ros::Point> input(4, 1);
    for (std::size_t i = 0; i < input.size(); ++i)
    {
        auto &point = input.points[i];
        point.x = 10.f + i;
        point.y = 2.f;
        point.z = 1.f;
        point.intensity = 20.f + i;
        point.reflectivity = 65535 - i;
        point.ring = 42 - i;
        point.t = 1000000 * (i + 1);
    }
    input.points[1].x = std::numeric_limits<float>::quiet_NaN();
    input.points[2].x = input.points[2].y = input.points[2].z = 0.f;
    sensor_msgs::PointCloud2::Ptr message(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(input, *message);

    Preprocess preprocess;
    preprocess.lidar_type = OUSTER;
    preprocess.blind = 0.1;
    preprocess.lidar_sensor_z_offset = 0.03618;
    PointCloudXYZI::Ptr output(new PointCloudXYZI());
    preprocess.process(message, output);

    ASSERT_EQ(2u, output->size());
    for (std::size_t i = 0; i < output->size(); ++i)
    {
        const std::size_t source = i == 0 ? 0 : 3;
        const auto point = makePublishedPoint(output->points[i], output->points[i], 100.0);
        EXPECT_FLOAT_EQ(input.points[source].x, point.x);
        EXPECT_NEAR(input.points[source].z - preprocess.lidar_sensor_z_offset, point.z, 1e-6);
        EXPECT_FLOAT_EQ(input.points[source].intensity, point.intensity);
        EXPECT_EQ(input.points[source].reflectivity, point.reflectivity);
        EXPECT_EQ(input.points[source].ring, point.ring);
        EXPECT_EQ(source, point.source_index);
        EXPECT_DOUBLE_EQ(100.0 + input.points[source].t * 1e-9, point.time);
    }
}

TEST(PointCloudPublishing, RawCoordinatesUseProcessedAttributes)
{
    PointType raw;
    raw.x = 1.f;
    raw.y = 2.f;
    raw.z = 3.f;
    raw.normal_z = 0.f;
    PointType corrected = raw;
    corrected.x = 4.f;
    corrected.y = 5.f;
    corrected.z = 6.f;
    corrected.intensity = 123.f;
    corrected.normal_z = 87.f;
    corrected.data_n[3] = 65535.f;
    corrected.data_c[2] = 127.f;
    corrected.normal_x = 901.f;
    corrected.curvature = 12.5f;

    pcl::PointCloud<PointXYZIReflectivity> cloud(2, 1);
    cloud.points[0] = makePublishedPoint(raw, corrected, 100.0);
    cloud.points[1] = makePublishedPoint(corrected, corrected, 100.0);
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    ASSERT_EQ(9u, message.fields.size());
    EXPECT_EQ(sensor_msgs::PointField::FLOAT32, message.fields[3].datatype);  // intensity
    EXPECT_EQ(sensor_msgs::PointField::UINT16, message.fields[4].datatype);   // reflectivity
    EXPECT_EQ(sensor_msgs::PointField::FLOAT32, message.fields[5].datatype); // corrected_intensity
    EXPECT_EQ(sensor_msgs::PointField::FLOAT64, message.fields[6].datatype); // time
    EXPECT_EQ(sensor_msgs::PointField::UINT32, message.fields[7].datatype);  // source_index
    EXPECT_EQ(sensor_msgs::PointField::UINT8, message.fields[8].datatype);   // ring

    pcl::PointCloud<PointXYZIReflectivity> decoded;
    pcl::fromROSMsg(message, decoded);
    ASSERT_EQ(2u, decoded.size());
    for (const auto &point : decoded.points)
    {
        EXPECT_FLOAT_EQ(123.f, point.intensity);
        EXPECT_EQ(65535u, point.reflectivity);
        EXPECT_FLOAT_EQ(87.f, point.corrected_intensity);
        EXPECT_DOUBLE_EQ(100.0125, point.time);
        EXPECT_EQ(901u, point.source_index);
        EXPECT_EQ(127u, point.ring);
    }
    EXPECT_FLOAT_EQ(raw.x, decoded.points[0].x);
    EXPECT_FLOAT_EQ(raw.y, decoded.points[0].y);
    EXPECT_FLOAT_EQ(raw.z, decoded.points[0].z);
    EXPECT_FLOAT_EQ(corrected.x, decoded.points[1].x);
    EXPECT_FLOAT_EQ(corrected.y, decoded.points[1].y);
    EXPECT_FLOAT_EQ(corrected.z, decoded.points[1].z);
}

TEST(PointCloudPublishing, LegacyCloudKeepsFieldsAndUsesSourceRing)
{
    PointCloudXYZI source(2, 2);
    PointCloudXYZI transformed(2, 2);
    for (std::size_t i = 0; i < source.size(); ++i)
    {
        source.points[i].data_c[2] = 127 - i;
        transformed.points[i].x = 10.f + i;
        transformed.points[i].intensity = 30.f + i;
    }
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(transformed, message);
    const auto original = message;
    addRingField(source, message);

    ASSERT_EQ(original.fields.size() + 1, message.fields.size());
    EXPECT_EQ("ring", message.fields.back().name);
    EXPECT_EQ(sensor_msgs::PointField::UINT8, message.fields.back().datatype);
    EXPECT_EQ(original.point_step + sizeof(float), message.point_step);
    EXPECT_EQ(message.row_step * message.height, message.data.size());
    sensor_msgs::PointCloud2ConstIterator<std::uint8_t> ring(message, "ring");
    for (std::size_t i = 0; i < source.size(); ++i, ++ring)
    {
        EXPECT_EQ(127 - i, *ring);
        EXPECT_EQ(0, std::memcmp(original.data.data() + i * original.point_step,
                                 message.data.data() + i * message.point_step,
                                 original.point_step));
    }
}

TEST(PointCloudPublishing, EmptyCloudAcceptsRingField)
{
    PointCloudXYZI source;
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(source, message);
    addRingField(source, message);
    EXPECT_TRUE(message.data.empty());
    EXPECT_EQ("ring", message.fields.back().name);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
