#pragma once
#include "common.h"
#include <fstream>
#include <string>

/** \brief Class realizing a linear mapping from vertical point angle to the corresponding scan ring.
 *
 */
class MultiScanMapper
{
public:
   /** \brief Construct a new multi scan mapper instance.
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
   MultiScanMapper(const float &lowerBound = -15,
                   const float &upperBound = 15,
                   const uint16_t &nScanRings = 16);

   const float &getLowerBound() { return _lowerBound; }
   const float &getUpperBound() { return _upperBound; }
   const uint16_t &getNumberOfScanRings() { return _nScanRings; }

   /** \brief Set mapping parameters.
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
   void set(const float &lowerBound,
            const float &upperBound,
            const uint16_t &nScanRings);

   /** \brief Map the specified vertical point angle to its ring ID.
   *
   * @param angle the vertical point angle (in rad)
   * @return the ring ID
   */
   int getRingForAngle(const float &angle);

   /** Multi scan mapper for Velodyne VLP-16 according to data sheet. */
   static inline MultiScanMapper Velodyne_VLP_16() { return MultiScanMapper(-15, 15, 16); };

   /** Multi scan mapper for Velodyne HDL-32 according to data sheet. */
   static inline MultiScanMapper Velodyne_HDL_32() { return MultiScanMapper(-30.67f, 10.67f, 32); };

   /** Multi scan mapper for Velodyne HDL-64E according to data sheet. */
   static inline MultiScanMapper Velodyne_HDL_64E() { return MultiScanMapper(-24.9f, 2, 64); };

   static inline MultiScanMapper Velodyne_VLP_32() { return MultiScanMapper(-15.639f, 10.333f, 32); };

private:
   float _lowerBound;    ///< the vertical angle of the first scan ring
   float _upperBound;    ///< the vertical angle of the last scan ring
   uint16_t _nScanRings; ///< number of scan rings
   float _factor;        ///< linear interpolation factor
};

/** \brief Class for registering point clouds received from multi-laser lidars.
 *
 */
class MultiScanRegistration
{
public:
   MultiScanRegistration(const MultiScanMapper &scanMapper = MultiScanMapper());

   /** \brief Setup component in active mode.
   *
   * @param node the ROS node handle
   * @param privateNode the private ROS node handle
   */
   bool setupLidarType(std::string lidarName = "VLP-32");

   /** \brief Process a new input cloud.
   *
   * @param laserCloudIn the new input cloud to process
   * @param scanTime the scan (message) timestamp
   */
   void process(const pcl::PointCloud<pcl::PointXYZI> &laserCloudIn, PointCloud::Ptr laserCloudOut, const double timeStamp);

private:
   float scanPeriod = 0.05;
   int _systemDelay = 20;       ///< system startup delay counter
   MultiScanMapper _scanMapper; ///< mapper for mapping vertical point angles to scan ring IDs
   std::vector<PointCloud> _laserCloudScans;
   // PointCloud::Ptr _subLaserCloud; ///< input cloud message subscriber
};
