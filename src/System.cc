/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <boost/variant/detail/backup_holder.hpp>

namespace ORB_SLAM2
{

	System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
		const bool bUseViewer) :mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false), mbActivateLocalizationMode(false),
		mbDeactivateLocalizationMode(false)
	{
		// Output welcome message
		cout << endl <<
			"ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
			"This program comes with ABSOLUTELY NO WARRANTY;" << endl <<
			"This is free software, and you are welcome to redistribute it" << endl <<
			"under certain conditions. See LICENSE.txt." << endl << endl;

		cout << "Input sensor was set to: ";

		if (mSensor == MONOCULAR)
			cout << "Monocular" << endl;
		else if (mSensor == STEREO)
			cout << "Stereo" << endl;
		else if (mSensor == RGBD)
			cout << "RGB-D" << endl;

		//Check settings file
		cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
		if (!fsSettings.isOpened())
		{
			cerr << "Failed to open settings file at: " << strSettingsFile << endl;
			exit(-1);
		}


		//Load ORB Vocabulary
		cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

		mpVocabulary = new ORBVocabulary();
		bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
		if (!bVocLoad)
		{
			cerr << "Wrong path to vocabulary. " << endl;
			cerr << "Falied to open at: " << strVocFile << endl;
			exit(-1);
		}
		cout << "Vocabulary loaded!" << endl << endl;

		//Create KeyFrame Database
		mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

		//Create the Map
		mpMap = new Map();

		//Create Drawers. These are used by the Viewer
		mpFrameDrawer = new FrameDrawer(mpMap);
		mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

		//Initialize the Tracking thread
		//(it will live in the main thread of execution, the one that called this constructor)
		mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
			mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

		//Initialize the Local Mapping thread and launch
		mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR);
		mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

		//Initialize the Loop Closing thread and launch
		mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR);
		mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

		//Initialize the Viewer thread and launch
		if (bUseViewer)
		{
			mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
			mptViewer = new thread(&Viewer::Run, mpViewer);
			mpTracker->SetViewer(mpViewer);
		}

		//Set pointers between threads
		mpTracker->SetLocalMapper(mpLocalMapper);
		mpTracker->SetLoopClosing(mpLoopCloser);

		mpLocalMapper->SetTracker(mpTracker);
		mpLocalMapper->SetLoopCloser(mpLoopCloser);

		mpLoopCloser->SetTracker(mpTracker);
		mpLoopCloser->SetLocalMapper(mpLocalMapper);
	}

	cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
	{
		if (mSensor != STEREO)
		{
			cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
			exit(-1);
		}

		// Check mode change
	{
		unique_lock<mutex> lock(mMutexMode);
		if (mbActivateLocalizationMode)
		{
			mpLocalMapper->RequestStop();

			// Wait until Local Mapping has effectively stopped
			while (!mpLocalMapper->isStopped())
			{
				usleep(1000);
			}

			mpTracker->InformOnlyTracking(true);
			mbActivateLocalizationMode = false;
		}
		if (mbDeactivateLocalizationMode)
		{
			mpTracker->InformOnlyTracking(false);
			mpLocalMapper->Release();
			mbDeactivateLocalizationMode = false;
		}
	}

	// Check reset
	{
		unique_lock<mutex> lock(mMutexReset);
		if (mbReset)
		{
			mpTracker->Reset();
			mbReset = false;
		}
	}

	cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp);

	unique_lock<mutex> lock2(mMutexState);
	mTrackingState = mpTracker->mState;
	mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
	mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
	return Tcw;
	}

	cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
	{
		if (mSensor != RGBD)
		{
			cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
			exit(-1);
		}

		// Check mode change
	{
		unique_lock<mutex> lock(mMutexMode);
		if (mbActivateLocalizationMode)
		{
			mpLocalMapper->RequestStop();

			// Wait until Local Mapping has effectively stopped
			while (!mpLocalMapper->isStopped())
			{
				usleep(1000);
			}

			mpTracker->InformOnlyTracking(true);
			mbActivateLocalizationMode = false;
		}
		if (mbDeactivateLocalizationMode)
		{
			mpTracker->InformOnlyTracking(false);
			mpLocalMapper->Release();
			mbDeactivateLocalizationMode = false;
		}
	}

	// Check reset
	{
		unique_lock<mutex> lock(mMutexReset);
		if (mbReset)
		{
			mpTracker->Reset();
			mbReset = false;
		}
	}

	cv::Mat Tcw = mpTracker->GrabImageRGBD(im, depthmap, timestamp);

	unique_lock<mutex> lock2(mMutexState);
	mTrackingState = mpTracker->mState;
	mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
	mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
	return Tcw;
	}

	cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
	{
		//mpLoopCloser->loop_detected = false;
		//mpTracker->mCurrentFrame.is_keyframe = false;

		if (mSensor != MONOCULAR)
		{
			cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
			exit(-1);
		}

		// Check mode change
	{
		unique_lock<mutex> lock(mMutexMode);
		if (mbActivateLocalizationMode)
		{
			mpLocalMapper->RequestStop();

			// Wait until Local Mapping has effectively stopped
			while (!mpLocalMapper->isStopped())
			{
				usleep(1000);
			}

			mpTracker->InformOnlyTracking(true);
			mbActivateLocalizationMode = false;
		}
		if (mbDeactivateLocalizationMode)
		{
			mpTracker->InformOnlyTracking(false);
			mpLocalMapper->Release();
			mbDeactivateLocalizationMode = false;
		}
	}

	// Check reset
	{
		unique_lock<mutex> lock(mMutexReset);
		if (mbReset)
		{
			mpTracker->Reset();
			mbReset = false;
		}
	}

	cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp);

	unique_lock<mutex> lock2(mMutexState);
	mTrackingState = mpTracker->mState;
	mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
	mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

	return Tcw;
	}

	void System::ActivateLocalizationMode()
	{
		unique_lock<mutex> lock(mMutexMode);
		mbActivateLocalizationMode = true;
	}

	void System::DeactivateLocalizationMode()
	{
		unique_lock<mutex> lock(mMutexMode);
		mbDeactivateLocalizationMode = true;
	}

	bool System::MapChanged()
	{
		static int n = 0;
		int curn = mpMap->GetLastBigChangeIdx();
		if (n < curn)
		{
			n = curn;
			return true;
		}
		else
			return false;
	}

	void System::Reset()
	{
		unique_lock<mutex> lock(mMutexReset);
		mbReset = true;
	}

	void System::Shutdown()
	{
		mpLocalMapper->RequestFinish();
		mpLoopCloser->RequestFinish();
		if (mpViewer)
		{
			mpViewer->RequestFinish();
			while (!mpViewer->isFinished())
				usleep(5000);
		}

		// Wait until all thread have effectively stopped
		while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
		{
			usleep(5000);
		}

		if (mpViewer)
			pangolin::BindToContext("ORB-SLAM2: Map Viewer");
	}

	void System::SaveTrajectoryTUM(const string &filename)
	{
		cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
		if (mSensor == MONOCULAR)
		{
			cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
			return;
		}

		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		cv::Mat Two = vpKFs[0]->GetPoseInverse();

		ofstream f;
		f.open(filename.c_str());
		f << fixed;

		// Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
		// We need to get first the keyframe pose and then concatenate the relative transformation.
		// Frames not localized (tracking failure) are not saved.

		// For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
		// which is true when tracking failed (lbL).
		list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
		list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
		list<bool>::iterator lbL = mpTracker->mlbLost.begin();
		for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
			lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++, lbL++)
		{
			if (*lbL)
				continue;

			KeyFrame* pKF = *lRit;

			cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

			// If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
			while (pKF->isBad())
			{
				Trw = Trw*pKF->mTcp;
				pKF = pKF->GetParent();
			}

			Trw = Trw*pKF->GetPose()*Two;

			cv::Mat Tcw = (*lit)*Trw;
			cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
			cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

			vector<float> q = Converter::toQuaternion(Rwc);

			f << setprecision(6) << *lT << " "
				<< setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) <<
				" " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
		}
		f.close();
		cout << endl << "trajectory saved!" << endl;
	}


	void System::SaveKeyFrameTrajectoryTUM(const string &filename)
	{
		cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		//cv::Mat Two = vpKFs[0]->GetPoseInverse();

		ofstream f;
		f.open(filename.c_str());
		f << fixed;

		for (size_t i = 0; i < vpKFs.size(); i++)
		{
			KeyFrame* pKF = vpKFs[i];

			// pKF->SetPose(pKF->GetPose()*Two);

			if (pKF->isBad())
				continue;

			cv::Mat R = pKF->GetRotation().t();
			vector<float> q = Converter::toQuaternion(R);
			cv::Mat t = pKF->GetCameraCenter();
			f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
				<< " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

		}

		f.close();
		cout << endl << "trajectory saved!" << endl;
	}

	void System::SaveTrajectoryKITTI(const string &filename)
	{
		cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
		if (mSensor == MONOCULAR)
		{
			cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
			return;
		}

		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		cv::Mat Two = vpKFs[0]->GetPoseInverse();

		ofstream f;
		f.open(filename.c_str());
		f << fixed;

		// Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
		// We need to get first the keyframe pose and then concatenate the relative transformation.
		// Frames not localized (tracking failure) are not saved.

		// For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
		// which is true when tracking failed (lbL).
		list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
		list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
		for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(), lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++)
		{
			ORB_SLAM2::KeyFrame* pKF = *lRit;

			cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

			while (pKF->isBad())
			{
				//  cout << "bad parent" << endl;
				Trw = Trw*pKF->mTcp;
				pKF = pKF->GetParent();
			}

			Trw = Trw*pKF->GetPose()*Two;

			cv::Mat Tcw = (*lit)*Trw;
			cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
			cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

			f << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2) << " " << twc.at<float>(0) << " " <<
				Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " " << twc.at<float>(1) << " " <<
				Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " " << twc.at<float>(2) << endl;
		}
		f.close();
		cout << endl << "trajectory saved!" << endl;
	}


	void System::SaveGridMapTUM(const string &filename)
	{

		cout << endl << "Saving grid map to " << filename << " ..." << endl;
		vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		ofstream f;
		f.open(filename.c_str());
		//f << fixed;

		f << "P2" << endl;

		// Init Grid Statictics
		// We set the resolution as 10mm, all points and keyframes are in the range of (-3, -4) to (3, 1),
		// so the grid map size is 600x500
		const float upper_left_x = -1.5;
		const float upper_left_y = -2.5;
		const int resolution = 10;
		const int h = 300;
		const int w = 450;
		double grid_occup[w][h];
		double grid_visit[w][h];

		memset(grid_occup, 0, sizeof(grid_occup[0][0]) * w * h);
		memset(grid_visit, 0, sizeof(grid_visit[0][0]) * w * h);

		f << w << " " << h << endl;
		f << 255 << endl;

		for (size_t i = 0; i<vpMPs.size(); i++)
		{
			MapPoint* pMP = vpMPs[i];

			if (pMP->isBad())
				continue;
			// Get the grid position of the map point
			cv::Mat MPPositions = pMP->GetWorldPos();
			float mp_pos_x = MPPositions.at<float>(0);
			float mp_pos_y = MPPositions.at<float>(1);

			int mp_pos_grid_x = ((int)((mp_pos_x - upper_left_x) * 1000)) / resolution;
			int mp_pos_grid_y = ((int)((mp_pos_y - upper_left_y) * 1000)) / resolution;

			if (mp_pos_grid_x < 0 || mp_pos_grid_x >= w)
				continue;

			if (mp_pos_grid_y < 0 || mp_pos_grid_y >= h)
				continue;

			// Increment the occupency account of the grid cell where map point is located
			grid_occup[mp_pos_grid_x][mp_pos_grid_y]++;

			// Get all KeyFrames that observes the map point
			std::map<KeyFrame*, size_t> obsFrames = pMP->GetObservations();
			std::map<KeyFrame*, size_t>::iterator it;

			//cout << "----------------------" << endl;
			for (it = obsFrames.begin(); it != obsFrames.end(); it++)
			{
				KeyFrame* oKF = it->first;
				if (oKF->isBad())
					continue;

				// Get the grid position of the KeyFrame
				cv::Mat t = oKF->GetCameraCenter();
				float okf_pos_x = t.at<float>(0);
				float okf_pos_y = t.at<float>(1);
				int okf_pos_grid_x = ((int)((okf_pos_x - upper_left_x) * 1000)) / resolution;
				int okf_pos_grid_y = ((int)((okf_pos_y - upper_left_y) * 1000)) / resolution;

				if (okf_pos_grid_x < 0 || okf_pos_grid_x >= w)
					continue;

				if (okf_pos_grid_y < 0 || okf_pos_grid_y >= h)
					continue;

				//cout << okf_pos_grid_x << " " << okf_pos_grid_y << endl;

				// Get all grid cell that the line between keyframe and map point pass through
				int x0 = okf_pos_grid_x;
				int y0 = okf_pos_grid_y;
				int x1 = mp_pos_grid_x;
				int y1 = mp_pos_grid_y;
				bool steep = (abs(y1 - y0) > abs(x1 - x0));
				if (steep)
				{
					x0 = okf_pos_grid_y;
					y0 = okf_pos_grid_x;
					x1 = mp_pos_grid_y;
					y1 = mp_pos_grid_x;
				}
				if (x0 > x1)
				{
					x0 = mp_pos_grid_y;
					x1 = okf_pos_grid_y;
					y0 = mp_pos_grid_x;
					y1 = okf_pos_grid_x;
				}
				int deltax = x1 - x0;
				int deltay = abs(y1 - y0);
				double error = 0;
				double deltaerr = ((double)deltay) / ((double)deltax);
				int y = y0;
				int ystep = (y0 < y1) ? 1 : -1;
				for (int x = x0; x <= x1; x++)
				{
					if (steep)
					{
						grid_visit[y][x]++;
					}
					else
					{
						grid_visit[x][y]++;
					}
					error = error + deltaerr;
					if (error >= 0.5)
					{
						y = y + ystep;
						error = error - 1.0;
					}
				}
			}
		}

		for (int i = 0; i<h; i++)
		{
			for (int j = 0; j<w; j++)
			{
				if (grid_visit[j][i] == 0)
				{
					f << 230 << " ";
					continue;
				}
				double r = grid_occup[j][i] / grid_visit[j][i];
				int grey = (int)(r * 255);
				if (grey > 0)
					grey += 100;
				if (grey > 255)
					grey = 255;
				f << 255 - grey << " ";

			}
			f << endl;
		}
		f.close();
		cout << endl << "Grid map saved!" << endl;
	}

	void System::Save2dMapPointsTUM(const string &filename, const int x, const int y)
	{
		cout << endl << "Saving 2d map points to " << filename << " ..." << endl;
		cout << endl << "x = " << x << ", y = " << y << endl;

		vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		ofstream f;
		f.open(filename.c_str());
		f << fixed;

		for (size_t i = 0; i<vpMPs.size(); i++)
		{
			MapPoint* pMP = vpMPs[i];

			if (pMP->isBad())
				continue;

			cv::Mat MPPositions = pMP->GetWorldPos();

			f << setprecision(7) << " " << MPPositions.at<float>(x) << " " << MPPositions.at<float>(y) << endl;
		}

		f.close();
		cout << endl << "2d Map Points saved!" << endl;
	}

	int System::GetTrackingState()
	{
		unique_lock<mutex> lock(mMutexState);
		return mTrackingState;
	}

	vector<MapPoint*> System::GetTrackedMapPoints()
	{
		unique_lock<mutex> lock(mMutexState);
		return mTrackedMapPoints;
	}

	vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
	{
		unique_lock<mutex> lock(mMutexState);
		return mTrackedKeyPointsUn;
	}

} //namespace ORB_SLAM
