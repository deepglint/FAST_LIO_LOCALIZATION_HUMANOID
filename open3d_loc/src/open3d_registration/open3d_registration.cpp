#include "open3d_registration/open3d_registration.h"

namespace pcd_tools
{

    open3d::pipelines::registration::RegistrationResult RegistrationFpfh(
        std::shared_ptr<open3d::geometry::PointCloud> source,
        std::shared_ptr<open3d::geometry::PointCloud> target,
        std::shared_ptr<open3d::pipelines::registration::Feature> source_fpfh,
        std::shared_ptr<open3d::pipelines::registration::Feature> target_fpfh,
        float voxel_size,
        open3d::utility::optional<unsigned int> seed_,
        bool mutual_filter)
    {

        float distance_threshold = 1.5 * voxel_size;

        // Prepare checkers
        std::vector<std::reference_wrapper<
            const open3d::pipelines::registration::CorrespondenceChecker>>
            correspondence_checker;
        auto correspondence_checker_edge_length =
            open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(
                0.9);
        auto correspondence_checker_distance =
            open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(
                distance_threshold);

        correspondence_checker.push_back(correspondence_checker_edge_length);
        correspondence_checker.push_back(correspondence_checker_distance);

        open3d::pipelines::registration::RegistrationResult registration_result;
        registration_result = open3d::pipelines::registration::
            RegistrationRANSACBasedOnFeatureMatching(
                *source, *target, *source_fpfh, *target_fpfh,
                mutual_filter, distance_threshold,
                open3d::pipelines::registration::
                    TransformationEstimationPointToPoint(false),
                4 /*最小3*/, correspondence_checker,
                open3d::pipelines::registration::RANSACConvergenceCriteria(1000000, 0.999), seed_);
        return registration_result;
    }

    open3d::pipelines::registration::RegistrationResult RegistrationIcp(
        std::shared_ptr<open3d::geometry::PointCloud> source,
        std::shared_ptr<open3d::geometry::PointCloud> target,
        float icp_distance_threshold, Eigen::Matrix4d init_matrix, int icp_method, int icp_iteration)
    {
        std::shared_ptr<open3d::geometry::PointCloud> source_transformed(new open3d::geometry::PointCloud);
        *source_transformed = *source;
        source_transformed->Transform(init_matrix);

        auto _criteria_icp =
            open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, icp_iteration);
        open3d::pipelines::registration::RegistrationResult registration_result;

        if (icp_method == 0)
        {
            registration_result = open3d::pipelines::registration::RegistrationICP(
                *source_transformed, *target, icp_distance_threshold, Eigen::Matrix4d::Identity(),
                open3d::pipelines::registration::TransformationEstimationPointToPoint(), _criteria_icp);
        }
        else if (icp_method == 1)
        {
            registration_result = open3d::pipelines::registration::RegistrationICP(
                *source_transformed, *target, icp_distance_threshold, Eigen::Matrix4d::Identity(),
                open3d::pipelines::registration::TransformationEstimationPointToPlane(), _criteria_icp);
        }
        else if (icp_method == 2)
        {
            registration_result = open3d::pipelines::registration::RegistrationGeneralizedICP(
                *source_transformed, *target, icp_distance_threshold, Eigen::Matrix4d::Identity(),
                open3d::pipelines::registration::TransformationEstimationForGeneralizedICP(), _criteria_icp);
        }
        return registration_result;
    }

    Eigen::Matrix4d RegistrationMultiScaleIcp(std::shared_ptr<open3d::geometry::PointCloud> source,
                                              std::shared_ptr<open3d::geometry::PointCloud> target,
                                              double voxel_size, int icp_method, std::vector<double> scale)
    {
        struct PcdPair
        {
            std::shared_ptr<open3d::geometry::PointCloud> pcd_src;
            std::shared_ptr<open3d::geometry::PointCloud> pcd_tgt;
            double voxel_size_;
            double icp_threshold_;
        };

        std::chrono::high_resolution_clock::time_point preprocess_all_s = std::chrono::high_resolution_clock::now();
        std::vector<PcdPair> vec_pcd_pair;

        for (std::size_t scale_i = 0; scale_i < scale.size(); ++scale_i)
        {
            PcdPair pair_;
            pair_.voxel_size_ = voxel_size * scale[scale_i];
            pair_.icp_threshold_ = pair_.voxel_size_ * 1.3;
            vec_pcd_pair.push_back(pair_);
        }

        int num_pair = vec_pcd_pair.size();
        // vec_pcd
        auto pcd_preprocess = [&](int pcd_i)
        {
            std::chrono::high_resolution_clock::time_point preprocess_s = std::chrono::high_resolution_clock::now();
            double voxel_size_ = vec_pcd_pair[pcd_i].voxel_size_;
            std::cout << "scale i: " << pcd_i << " voxel_size: " << voxel_size_ << std::endl;
            vec_pcd_pair[pcd_i].pcd_src = source->VoxelDownSample(voxel_size_);
            vec_pcd_pair[pcd_i].pcd_tgt = target->VoxelDownSample(voxel_size_);
            vec_pcd_pair[pcd_i].pcd_tgt->EstimateNormals(
                open3d::geometry::KDTreeSearchParamHybrid(voxel_size_ * 2, 30));

            std::chrono::high_resolution_clock::time_point preprocess_e = std::chrono::high_resolution_clock::now();
            auto preprocess_cost = std::chrono::duration_cast<std::chrono::milliseconds>(preprocess_e - preprocess_s).count();
        };

        std::vector<std::thread> thread_preprocess;
        for (int i = 0; i < num_pair; ++i)
        {
            thread_preprocess.push_back(std::thread(pcd_preprocess, i));
        }
        for (auto &t : thread_preprocess)
        {
            t.join();
        }
        std::chrono::high_resolution_clock::time_point preprocess_all_e = std::chrono::high_resolution_clock::now();
        auto preprocess_all_cost = std::chrono::duration_cast<std::chrono::milliseconds>(preprocess_all_e - preprocess_all_s).count();

        // /*配准*/
        open3d::pipelines::registration::RegistrationResult registration_result;
        int count_icp = 0;
        Eigen::Matrix4d matrix_icp = Eigen::Matrix4d::Identity();
        // 使用最大的阈值开始多次配准，每次阈值为上次的0.6，直到阈值小于0.5*voxel_size
        std::chrono::high_resolution_clock::time_point icp_s = std::chrono::high_resolution_clock::now();

        double icp_threshold_current;
        for (int icp_i = 0; icp_i < num_pair; ++icp_i)
        {
            count_icp += 1;
            auto src = vec_pcd_pair[num_pair - 1 - icp_i].pcd_src;
            auto tgt = vec_pcd_pair[num_pair - 1 - icp_i].pcd_tgt;
            icp_threshold_current = vec_pcd_pair[num_pair - 1 - icp_i].icp_threshold_;
            registration_result = RegistrationIcp(src, tgt, icp_threshold_current, matrix_icp, icp_method, 30);
            matrix_icp = registration_result.transformation_ * matrix_icp;
        }
        std::chrono::high_resolution_clock::time_point icp_e = std::chrono::high_resolution_clock::now();
        auto icp_cost = std::chrono::duration_cast<std::chrono::milliseconds>(icp_e - icp_s).count();

        return matrix_icp;
    }

    open3d::pipelines::registration::RegistrationResult RegistrationEvaluate(const std::shared_ptr<open3d::geometry::PointCloud> src, const std::shared_ptr<open3d::geometry::PointCloud> tgt, double voxel_size, Eigen::Matrix4d transformation)
    {
        auto src_down = src->VoxelDownSample(voxel_size);
        auto tgt_down = tgt->VoxelDownSample(voxel_size);
        return open3d::pipelines::registration::EvaluateRegistration(*src_down, *tgt_down, voxel_size * 1.5, transformation);
    }

    Open3dRegistration::Open3dRegistration()
    {
        std::cout << "constructor" << std::endl;
        ResetParam();
    }
    Open3dRegistration::Open3dRegistration(const Open3dRegistration &o3d_reg)
    {
        std::cout << "copy constructor" << std::endl;
        ResetParam();
        // from config
        this->neighbors = o3d_reg.neighbors;
        this->neighbors = o3d_reg.std_ratio;
        this->voxel_size = o3d_reg.voxel_size;
        this->icp_method = o3d_reg.icp_method;
        this->seed_ = o3d_reg.seed_;
        this->threshold_overlap = o3d_reg.threshold_overlap;
        this->mutual_filter = o3d_reg.mutual_filter;

        // from client
        this->path_source = o3d_reg.path_source;
        this->path_target = o3d_reg.path_target;
        this->statistical_filter_source = o3d_reg.statistical_filter_source;
        this->statistical_filter_target = o3d_reg.statistical_filter_target;
        this->use_fpfh = o3d_reg.use_fpfh;

        // something may changed
        if (nullptr != o3d_reg.source_ori)
        {
            this->source_ori.reset(new open3d::geometry::PointCloud);
            *this->source_ori = *o3d_reg.source_ori;
        }
        if (nullptr != o3d_reg.target_ori)
        {
            this->target_ori.reset(new open3d::geometry::PointCloud);
            *this->target_ori = *o3d_reg.target_ori;
        }
        if (nullptr != o3d_reg.source_fpfh)
        {
            this->source_fpfh.reset(new open3d::pipelines::registration::Feature);
            *this->source_fpfh = *o3d_reg.source_fpfh;
        }
        if (nullptr != o3d_reg.target_fpfh)
        {
            this->target_fpfh.reset(new open3d::pipelines::registration::Feature);
            *this->source_fpfh = *o3d_reg.target_fpfh;
        }
        if (nullptr != o3d_reg.CropBox_source)
        {
            this->CropBox_source.reset(new open3d::geometry::OrientedBoundingBox());
            *this->CropBox_source = *o3d_reg.CropBox_source;
        }
        if (nullptr != o3d_reg.CropBox_target)
        {
            this->CropBox_target.reset(new open3d::geometry::OrientedBoundingBox());
            *this->CropBox_target = *o3d_reg.CropBox_target;
        }
        this->overlap = o3d_reg.overlap;
        this->initial_matrix = o3d_reg.initial_matrix;
        this->fpfh_matrix = o3d_reg.fpfh_matrix;
        this->icp_matrix = o3d_reg.icp_matrix;
        this->final_transformation = o3d_reg.final_transformation;
        this->regresult = o3d_reg.regresult;
    }
    Open3dRegistration::~Open3dRegistration()
    {
        std::cout << "~Open3dRegistration" << std::endl;
    }
    void Open3dRegistration::ResetParam()
    {
        voxel_downsample = true;
        use_fpfh = true;
        use_icp = true;
        statistical_filter_source = true;
        statistical_filter_target = true;
        neighbors = 50;
        std_ratio = 3.0;
        path_source = "";
        path_target = "";
        voxel_size = 0.05;
        icp_iteration = 30;
        icp_method = 1;

        seed_ = open3d::utility::nullopt;
        mutual_filter = true;

        initial_matrix = Eigen::Matrix4d::Identity();
        fpfh_matrix = Eigen::Matrix4d::Identity();
        icp_matrix = Eigen::Matrix4d::Identity();
        final_transformation = Eigen::Matrix4d::Identity();

        source.reset(new open3d::geometry::PointCloud);

        target.reset(new open3d::geometry::PointCloud);
        source_ori.reset(new open3d::geometry::PointCloud);
        target_ori.reset(new open3d::geometry::PointCloud);

        source_fpfh = nullptr;
        target_fpfh = nullptr;

        CropBox_source.reset(new open3d::geometry::OrientedBoundingBox());
        CropBox_target.reset(new open3d::geometry::OrientedBoundingBox());

        regresult = open3d::pipelines::registration::RegistrationResult();
        overlap = 0.0;
        threshold_overlap = 0.8;
    }
    std::tuple<std::shared_ptr<open3d::geometry::PointCloud>,
               std::shared_ptr<open3d::pipelines::registration::Feature>>
    Open3dRegistration::PreprocessPointCloud(std::shared_ptr<open3d::geometry::PointCloud> pcd,
                                             float voxel_size,
                                             bool filter, /*true*/
                                             open3d::geometry::OrientedBoundingBox cropbox)
    {
        if (nullptr == pcd || pcd->IsEmpty())
        {
            std::cout << "no data in pcd" << std::endl;
            return std::make_tuple(pcd, std::make_shared<open3d::pipelines::registration::Feature>());
        }

        *pcd = pcd->RemoveNonFinitePoints(true, true);

        // 1、crop
        if (cropbox.extent_[0] > 0)
        {
            pcd = pcd->Crop(cropbox);
        }

        // 2、statistical filter
        if (filter)
        {
            auto sta_filered = pcd->RemoveStatisticalOutliers(neighbors, std_ratio, false);
            pcd = std::get<0>(sta_filered);
        }
        // 3、降采样
        if (voxel_downsample)
        {

            pcd = pcd->VoxelDownSample(voxel_size);
        }
        // 4、计算法向量

        pcd->EstimateNormals(
            open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 2, 30));
        /*
        https://github.com/isl-org/Open3D/blob/master/cpp/open3d/geometry/PointCloud.h#L239
        void OrientNormalsToAlignWithDirection(
        const Eigen::Vector3d& orientation_reference =
            Eigen::Vector3d(0.0, 0.0, 1.0));
        https://github.com/isl-org/Open3D/blob/7c62640441e3da18bcbe146723ed83ff544b2fbb/cpp/open3d/geometry/EstimateNormals.cpp#L338
        */
        // 指定法向量方向
        pcd->OrientNormalsToAlignWithDirection();

        // 5、计算fpfh特征
        auto pcd_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(
            *pcd,
            open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 5, 100));

        return std::make_tuple(pcd, pcd_fpfh);
    }

    bool Open3dRegistration::RegistrationPipeline()
    {
        std::cout << "RegistrationPipeline start" << std::endl;
        std::cout << "o3d_reg.source_ori size: " << source_ori->points_.size() << std::endl
                  << "o3d_reg.target_ori size: " << target_ori->points_.size() << std::endl;
        // 判断点云是否为空一定先判断指针是否为空
        if (source_ori == nullptr || source_ori->IsEmpty() || target_ori == nullptr || target_ori->IsEmpty())
        {
            std::cout << "no data in pcd need to be registration" << std::endl;
            return false;
        }
        source.reset(new open3d::geometry::PointCloud);
        target.reset(new open3d::geometry::PointCloud);
        *source = *source_ori;
        *target = *target_ori;
        std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
        source->Transform(initial_matrix);

        if (use_fpfh)
        {
            std::chrono::high_resolution_clock::time_point fpfh_time_s = std::chrono::high_resolution_clock::now();
            // 点云预处理
            std::tie(source, source_fpfh) = PreprocessPointCloud(source, voxel_size, statistical_filter_source, *CropBox_source);
            std::tie(target, target_fpfh) = PreprocessPointCloud(target, voxel_size, statistical_filter_target, *CropBox_target);

            // 粗配准
            open3d::pipelines::registration::RegistrationResult registration_result;
            registration_result = pcd_tools::RegistrationFpfh(source, target, source_fpfh, target_fpfh, voxel_size, seed_, true);
            fpfh_matrix = registration_result.transformation_;
            std::chrono::high_resolution_clock::time_point fpfh_time_e = std::chrono::high_resolution_clock::now();
        }

        if (use_icp)
        {
            std::chrono::high_resolution_clock::time_point icp_time_s = std::chrono::high_resolution_clock::now();
            source->Transform(fpfh_matrix);
            icp_matrix = pcd_tools::RegistrationMultiScaleIcp(source, target, voxel_size, icp_method);
            final_transformation = icp_matrix * fpfh_matrix * initial_matrix;
            std::chrono::high_resolution_clock::time_point icp_time_e = std::chrono::high_resolution_clock::now();
        }

        // 计算最终的结果
        regresult = RegistrationEvaluate(source_ori, target_ori, voxel_size, final_transformation);
        overlap = regresult.fitness_;
        std::chrono::high_resolution_clock::time_point total_time = std::chrono::high_resolution_clock::now();

        return true;
    }

    Eigen::Matrix4d Open3dRegistration::GetFinalMatrix()
    {
        return (icp_matrix * fpfh_matrix * initial_matrix);
    }

    void Open3dRegistration::PrintRegistrationResult(const open3d::pipelines::registration::RegistrationResult &result)
    {
        std::cout << "matrix:" << std::endl
                  << result.transformation_ << std::endl;
        std::cout << "inlier(correspondence_set size):"
                  << result.correspondence_set_.size() << std::endl;
        std::cout << "\nfitness_(For RANSAC: inlier ratio (# of inlier correspondences / # of all correspondences)): " << result.fitness_ << std::endl
                  << "RMSE of all inlier correspondences: " << result.inlier_rmse_ << std::endl;
    }

}