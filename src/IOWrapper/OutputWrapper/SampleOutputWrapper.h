/**
 * This file is part of DSO.
 *
 * Copyright 2016 Technical University of Munich and Intel.
 * Developed by Jakob Engel <engelj at in dot tum dot de>,
 * for more information see <http://vision.in.tum.de/dso>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * DSO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DSO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DSO. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include "boost/thread.hpp"

#include "FullSystem/HessianBlocks.h"
#include "IOWrapper/Output3DWrapper.h"
#include "util/FrameShell.h"
#include "util/MinimalImage.h"




namespace dso
{

class FrameHessian;
class CalibHessian;
class FrameShell;

namespace IOWrap
{

class SampleOutputWrapper: public Output3DWrapper
{
public:
    inline SampleOutputWrapper()
    {
        printf("OUT: Created SampleOutputWrapper\n");
    }

    virtual ~SampleOutputWrapper()
    {
        printf("OUT: Destroyed SampleOutputWrapper\n");
    }

    virtual void setParameter(CalibHessian* HCalib)
    {
        fx = HCalib->fxl();
        fy = HCalib->fyl();
        cx = HCalib->cxl();
        cy = HCalib->cyl();
        fxi = 1 / fx;
        fyi = 1 / fy;
        cxi = -cx / fx;
        cyi = -cy / fy;
    }

    virtual void publishGraph(
            const std::map<long, Eigen::Vector2i> &connectivity)
    {
        printf("OUT: got graph with %d edges\n", (int) connectivity.size());

        int maxWrite = 5;

        for (const std::pair<long, Eigen::Vector2i> &p : connectivity)
        {
            int idHost = p.first >> 32;
            int idTarget = p.first & 0xFFFFFFFF;
            printf("OUT: Example Edge %d -> %d has %d active and "
                    "%d marg residuals\n", idHost, idTarget, p.second[0],
                    p.second[1]);
            maxWrite--;
            if (maxWrite == 0)
                break;
        }
    }

    virtual void publishKeyframes(std::vector<FrameHessian*> &frames,
            bool final, CalibHessian* HCalib)
    {
        setParameter(HCalib);

        const std::string keyframeFilename = "keyframePoses.txt";
        const std::string pointcloudFilename = "pointCouldPositions.txt";
        std::ofstream keyframeFile, pointcloudFile;

        keyframeFile.open(keyframeFilename.c_str(), std::ios_base::app);
        keyframeFile << std::setprecision(15);

        pointcloudFile.open(pointcloudFilename.c_str(), std::ios_base::app);
        pointcloudFile << std::setprecision(15);

        // Point position in camera frame and world frame
        Eigen::Matrix<double, 4, 1> p_C_pointcloud;
        Eigen::Matrix<double, 4, 1> p_W_pointcloud;

        // Transformation matrix from camera frame to world frame
        Eigen::Matrix<double, 4, 4> T_W_C;

        float depth;

        for (FrameHessian* f : frames)
        {
            // Process the optimized keyframes (in final model).
            if (final == true)
            {
                if (!f->shell->poseValid)
                    continue;
                // Log the keyframe ID & poses in text file.
                keyframeFile << f->frameID << " " << f->shell->timestamp << " "
                        << f->shell->camToWorld.translation().transpose() << " "
                        << f->shell->camToWorld.so3().unit_quaternion().x()
                        << " "
                        << f->shell->camToWorld.so3().unit_quaternion().y()
                        << " "
                        << f->shell->camToWorld.so3().unit_quaternion().z()
                        << " "
                        << f->shell->camToWorld.so3().unit_quaternion().w()
                        << "\n";

                T_W_C = f->shell->camToWorld.matrix();

                // Display the keyframes.
                printf("OUT: KF %d (%s) (id %d, tme %f): %d active, "
                        "%d marginalized, %d immature points. CameraToWorld:\n",
                        f->frameID, final ? "final" : "non-final",
                        f->shell->incoming_id, f->shell->timestamp,
                        (int) f->pointHessians.size(),
                        (int) f->pointHessiansMarginalized.size(),
                        (int) f->immaturePoints.size());
                std::cout << T_W_C << "\n";

                // Log the point host ID & positions in text file.
                for (PointHessian* p : f->pointHessiansMarginalized)
                {
                    depth = 1.0f / p->idepth_scaled;
                    p_C_pointcloud[0] = (p->u * fxi + cxi) * depth;
                    p_C_pointcloud[1] = (p->v * fyi + cyi) * depth;
                    p_C_pointcloud[2] = depth * (1 + 2 * fxi *
                            (rand() / (float) RAND_MAX - 0.5f));
                    p_C_pointcloud[3] = 1;

                    p_W_pointcloud = T_W_C * p_C_pointcloud;

                    pointcloudFile << f->frameID
                            << " " << p_W_pointcloud.transpose().head(3)
                            << " " << p->u << " " << p->v
                            << " " << p->idepth_scaled
                            << " " << sqrt(1.0f / p->idepth_hessian)
                            << " " << p->numGoodResiduals << "\n";
                }
            }
        }
        keyframeFile.close();
        pointcloudFile.close();
    }

    virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib)
    {
        /*printf(
                "OUT: Current Frame %d (time %f, internal ID %d). CameraToWorld:\n",
                frame->incoming_id, frame->timestamp, frame->id);
        std::cout << frame->camToWorld.matrix3x4() << "\n";*/
    }

    virtual void pushLiveFrame(FrameHessian* image)
    {
        // can be used to get the raw image / intensity pyramid.
    }

    virtual void pushDepthImage(MinimalImageB3* image)
    {
        // can be used to get the raw image with depth overlay.
    }

    virtual bool needPushDepthImage()
    {
        return false;
    }

    virtual void pushDepthImageFloat(MinimalImageF* image, FrameHessian* KF)
    {
        printf("OUT: Predicted depth for KF %d (id %d, time %f,"
                " internal frame-ID %d). CameraToWorld:\n", KF->frameID,
                KF->shell->incoming_id, KF->shell->timestamp, KF->shell->id);
        std::cout << KF->shell->camToWorld.matrix3x4() << "\n";

        int maxWrite = 5;
        for (int y = 0; y < image->h; y++)
        {
            for (int x = 0; x < image->w; x++)
            {
                if (image->at(x, y) <= 0)
                    continue;

                printf("OUT: Example Idepth at pixel (%d,%d): %f.\n", x, y,
                        image->at(x, y));
                maxWrite--;
                if (maxWrite == 0)
                    break;
            }
            if (maxWrite == 0)
                break;
        }
    }

private:
    float fx, fy, cx, cy;
    float fxi, fyi, cxi, cyi;
};

}

}
