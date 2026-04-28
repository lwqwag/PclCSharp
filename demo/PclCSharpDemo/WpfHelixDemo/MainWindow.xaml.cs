using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using Microsoft.Win32;
using PclCSharp;
using PointCloudSharp;

/*
 * Copyright (c) 2024, PclCSharp Contributors
 * WPF demo using HelixToolkit for 3D point-cloud visualization.
 * Demonstrates operators from the PclCSharp wrapper:
 *   - IO        : load / save PCD / PLY / OBJ / TXT
 *   - Filter    : voxel / approx-voxel / uniform / passThrough / statistical / radius / sigma
 *   - Segmentation : euclidean cluster / region growing / modified region growing
 *   - SampleConsensus : RANSAC plane fitting
 *   - Util      : correctPlane / calculateRunout
 */

namespace WpfHelixDemo;

public partial class MainWindow : Window
{
    // ─── Working point clouds ────────────────────────────────────────────────
    private PointCloudXYZ _inputCloud = new();
    private PointCloudXYZ _outputCloud = new();

    // Cached plane coefficients from the last fitPlane call (a,b,c,d)
    private float[] _planeNormal = new float[4];

    // ─── Constructor ────────────────────────────────────────────────────────
    public MainWindow()
    {
        InitializeComponent();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Helpers
    // ═══════════════════════════════════════════════════════════════════════════

    /// <summary>
    /// Remove all children except the first (DefaultLights) from a HelixViewport3D.
    /// Visual3DCollection does not support RemoveRange, so we iterate in reverse.
    /// </summary>
    private static void ClearViewportChildren(HelixViewport3D viewport)
    {
        for (int i = viewport.Children.Count - 1; i >= 1; i--)
            viewport.Children.RemoveAt(i);
    }

    /// <summary>
    /// Convert a PointCloudXYZ into a Media3D Point3DCollection.
    /// </summary>
    private static Point3DCollection ToPoint3DCollection(PointCloudXYZ cloud)
    {
        var pts = new Point3DCollection(cloud.Size);
        for (int i = 0; i < cloud.Size; i++)
            pts.Add(new Point3D(cloud.GetX(i), cloud.GetY(i), cloud.GetZ(i)));
        return pts;
    }

    /// <summary>
    /// Display a point cloud in a HelixViewport3D using one or more PointsVisual3D.
    /// Points are coloured by Z value (blue→cyan→green→yellow→red rainbow).
    /// </summary>
    private static void ShowCloud(HelixViewport3D viewport, PointCloudXYZ cloud,
                                   int highlightMin = -1, int highlightMax = -1)
    {
        ClearViewportChildren(viewport);

        // Find Z range
        var minmax = new double[6];
        cloud.GetMinMaxXYZ(minmax);
        double zMin = minmax[4];
        double zRange = minmax[5] - minmax[4];
        if (zRange < 1e-9) zRange = 1.0;

        // We bucket points into 6 colour bands to achieve a rainbow effect
        // without needing per-point colour support.
        const int Bands = 6;
        var buckets = new List<Point3D>[Bands];
        for (int b = 0; b < Bands; b++) buckets[b] = [];

        for (int i = 0; i < cloud.Size; i++)
        {
            double t = (cloud.GetZ(i) - zMin) / zRange;      // 0..1
            int band = Math.Clamp((int)(t * Bands), 0, Bands - 1);
            buckets[band].Add(new Point3D(cloud.GetX(i), cloud.GetY(i), cloud.GetZ(i)));
        }

        // Rainbow colours per band: blue→indigo→cyan→green→yellow→red
        Color[] bandColors =
        [
            Color.FromRgb(0, 50, 255),
            Color.FromRgb(0, 160, 220),
            Color.FromRgb(0, 220, 100),
            Color.FromRgb(100, 220, 0),
            Color.FromRgb(220, 180, 0),
            Color.FromRgb(220, 50,  0),
        ];

        for (int b = 0; b < Bands; b++)
        {
            if (buckets[b].Count == 0) continue;
            var pv = new PointsVisual3D
            {
                Color = bandColors[b],
                Size  = 2,
                Points = new Point3DCollection(buckets[b])
            };
            viewport.Children.Add(pv);
        }

        // Highlight min / max points if indices given
        if (highlightMin >= 0 && highlightMin < cloud.Size)
        {
            var pt = new Point3D(cloud.GetX(highlightMin), cloud.GetY(highlightMin),
                                  cloud.GetZ(highlightMin));
            viewport.Children.Add(new PointsVisual3D
            {
                Color  = Colors.Cyan,
                Size   = 8,
                Points = new Point3DCollection([pt])
            });
        }
        if (highlightMax >= 0 && highlightMax < cloud.Size)
        {
            var pt = new Point3D(cloud.GetX(highlightMax), cloud.GetY(highlightMax),
                                  cloud.GetZ(highlightMax));
            viewport.Children.Add(new PointsVisual3D
            {
                Color  = Colors.Magenta,
                Size   = 8,
                Points = new Point3DCollection([pt])
            });
        }

        viewport.ZoomExtents();
    }

    /// <summary>
    /// Display multiple clusters with distinct colours (for segmentation results).
    /// <paramref name="clusterClouds"/> is a list of per-cluster PointCloudXYZ objects.
    /// </summary>
    private static void ShowClusters(HelixViewport3D viewport,
                                      IReadOnlyList<PointCloudXYZ> clusterClouds)
    {
        ClearViewportChildren(viewport);

        Color[] palette =
        [
            Colors.OrangeRed, Colors.DeepSkyBlue, Colors.LimeGreen,
            Colors.Gold, Colors.Orchid, Colors.Tomato, Colors.Aquamarine,
            Colors.HotPink, Colors.Chartreuse, Colors.DodgerBlue
        ];

        for (int c = 0; c < clusterClouds.Count; c++)
        {
            var cloud = clusterClouds[c];
            if (cloud.Size == 0) continue;
            var pts = ToPoint3DCollection(cloud);
            viewport.Children.Add(new PointsVisual3D
            {
                Color  = palette[c % palette.Length],
                Size   = 2,
                Points = pts
            });
        }

        viewport.ZoomExtents();
    }

    /// <summary>Update status bar text.</summary>
    private void SetStatus(string msg) => StatusText.Text = msg;

    /// <summary>Update the info text blocks.</summary>
    private void UpdateInfo()
    {
        InputInfoText.Text  = $"输入点云：{_inputCloud.Size:N0} 个点";
        OutputInfoText.Text = $"输出点云：{_outputCloud.Size:N0} 个点";
    }

    /// <summary>
    /// Show input cloud in the input viewport and refresh info.
    /// </summary>
    private void RefreshInput()
    {
        ShowCloud(InputViewport, _inputCloud);
        UpdateInfo();
    }

    /// <summary>
    /// Show output cloud in the output viewport and refresh info.
    /// </summary>
    private void RefreshOutput()
    {
        ShowCloud(OutputViewport, _outputCloud);
        UpdateInfo();
    }

    /// <summary>
    /// Generate a synthetic point cloud (disk + random noise outliers)
    /// so the demo is usable without any external file or native DLL.
    /// </summary>
    private static PointCloudXYZ GenerateSyntheticCloud(int diskPoints = 6_000,
                                                          int noisePoints = 200)
    {
        var cloud = new PointCloudXYZ();
        var rng = new Random(42);

        // Disk in the XY plane with z ≈ 0 + small noise
        for (int i = 0; i < diskPoints; i++)
        {
            double angle  = rng.NextDouble() * 2 * Math.PI;
            double r      = Math.Sqrt(rng.NextDouble()) * 2.0; // uniform on disk radius 2
            double x      = r * Math.Cos(angle);
            double y      = r * Math.Sin(angle);
            double z      = (rng.NextDouble() - 0.5) * 0.05;   // thin disk
            cloud.Push(x, y, z);
        }

        // Random outliers scattered above/below
        for (int i = 0; i < noisePoints; i++)
        {
            double x = (rng.NextDouble() - 0.5) * 6;
            double y = (rng.NextDouble() - 0.5) * 6;
            double z = (rng.NextDouble() > 0.5 ? 1 : -1) * (0.5 + rng.NextDouble() * 1.5);
            cloud.Push(x, y, z);
        }

        return cloud;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  IO Tab handlers
    // ═══════════════════════════════════════════════════════════════════════════

    private void LoadPcdBtn_Click(object sender, RoutedEventArgs e)
        => LoadFile("PCD 文件|*.pcd", path =>
        {
            _inputCloud.Clear();
            int ret = Io.loadPcdFile(path, _inputCloud.PointCloudXYZPointer);
            if (ret != 0)
                MessageBox.Show($"加载 PCD 失败，错误码：{ret}", "错误");
        });

    private void LoadPlyBtn_Click(object sender, RoutedEventArgs e)
        => LoadFile("PLY 文件|*.ply", path =>
        {
            _inputCloud.Clear();
            int ret = Io.loadPlyFile(path, _inputCloud.PointCloudXYZPointer);
            if (ret != 0)
                MessageBox.Show($"加载 PLY 失败，错误码：{ret}", "错误");
        });

    private void LoadObjBtn_Click(object sender, RoutedEventArgs e)
        => LoadFile("OBJ 文件|*.obj", path =>
        {
            _inputCloud.Clear();
            int ret = Io.loadObjFile(path, _inputCloud.PointCloudXYZPointer);
            if (ret != 0)
                MessageBox.Show($"加载 OBJ 失败，错误码：{ret}", "错误");
        });

    private void LoadTxtBtn_Click(object sender, RoutedEventArgs e)
        => LoadFile("TXT 文件|*.txt", path =>
        {
            _inputCloud.Clear();
            int ret = Io.loadTxtFile(path, _inputCloud.PointCloudXYZPointer);
            if (ret != 0)
                MessageBox.Show($"加载 TXT 失败，错误码：{ret}", "错误");
        });

    private void SavePcdBtn_Click(object sender, RoutedEventArgs e)
        => SaveFile("PCD 文件|*.pcd", path =>
            Io.savePcdFile(path, _outputCloud.PointCloudXYZPointer, 0));

    private void SavePlyBtn_Click(object sender, RoutedEventArgs e)
        => SaveFile("PLY 文件|*.ply", path =>
            Io.savePlyFile(path, _outputCloud.PointCloudXYZPointer, 0));

    private void GenSyntheticBtn_Click(object sender, RoutedEventArgs e)
    {
        _inputCloud = GenerateSyntheticCloud();
        RefreshInput();
        SetStatus($"已生成合成点云 – {_inputCloud.Size:N0} 个点");
    }

    // Shared helpers for open / save dialogs
    private void LoadFile(string filter, Action<string> load)
    {
        var dlg = new OpenFileDialog { Filter = filter };
        if (dlg.ShowDialog() != true) return;
        try
        {
            load(dlg.FileName);
            RefreshInput();
            SetStatus($"已加载：{System.IO.Path.GetFileName(dlg.FileName)} ({_inputCloud.Size:N0} pts)");
        }
        catch (Exception ex)
        {
            MessageBox.Show($"操作失败：{ex.Message}", "错误",
                            MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    private void SaveFile(string filter, Action<string> save)
    {
        if (_outputCloud.Size == 0)
        {
            MessageBox.Show("输出点云为空，请先执行一个算子。", "提示");
            return;
        }
        var dlg = new SaveFileDialog { Filter = filter };
        if (dlg.ShowDialog() != true) return;
        try
        {
            save(dlg.FileName);
            SetStatus($"已保存：{System.IO.Path.GetFileName(dlg.FileName)}");
        }
        catch (Exception ex)
        {
            MessageBox.Show($"保存失败：{ex.Message}", "错误",
                            MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Filter Tab handlers
    // ═══════════════════════════════════════════════════════════════════════════

    private bool EnsureInputCloud()
    {
        if (_inputCloud.Size > 0) return true;
        MessageBox.Show("请先在「IO」选项卡中加载或生成点云。", "提示");
        return false;
    }

    private void RunFilter(string name, Action filter)
    {
        if (!EnsureInputCloud()) return;
        try
        {
            _outputCloud.Clear();
            filter();
            RefreshOutput();
            SetStatus($"{name} 完成 – 输入 {_inputCloud.Size:N0} 点 → 输出 {_outputCloud.Size:N0} 点");
        }
        catch (Exception ex)
        {
            MessageBox.Show($"{name} 失败：{ex.Message}", "错误",
                            MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    private void VoxelBtn_Click(object sender, RoutedEventArgs e)
    {
        if (!double.TryParse(VoxelLeafTxt.Text, out double leaf)) leaf = 0.08;
        RunFilter("体素下采样", () =>
            Filter.voxelDownSample(_inputCloud.PointCloudXYZPointer, leaf,
                                    _outputCloud.PointCloudXYZPointer));
    }

    private void ApproxVoxelBtn_Click(object sender, RoutedEventArgs e)
    {
        if (!double.TryParse(ApproxVoxelLeafTxt.Text, out double leaf)) leaf = 0.08;
        RunFilter("近似体素下采样", () =>
            Filter.approximateVoxelDownSample(_inputCloud.PointCloudXYZPointer, leaf,
                                               _outputCloud.PointCloudXYZPointer));
    }

    private void UniformBtn_Click(object sender, RoutedEventArgs e)
    {
        if (!double.TryParse(UniformRadiusTxt.Text, out double r)) r = 0.1;
        RunFilter("均匀下采样", () =>
            Filter.uniformDownSample(_inputCloud.PointCloudXYZPointer, r,
                                      _outputCloud.PointCloudXYZPointer));
    }

    private void PassBtn_Click(object sender, RoutedEventArgs e)
    {
        string axis = (PassAxisCombo.SelectedItem as System.Windows.Controls.ComboBoxItem)
                      ?.Content?.ToString() ?? "z";
        if (!float.TryParse(PassMinTxt.Text, out float mn)) mn = -0.5f;
        if (!float.TryParse(PassMaxTxt.Text, out float mx)) mx =  0.5f;
        RunFilter("直通滤波", () =>
            Filter.passThroughFilter(_inputCloud.PointCloudXYZPointer, axis,
                                      mn, mx, 0, _outputCloud.PointCloudXYZPointer));
    }

    private void StaBtn_Click(object sender, RoutedEventArgs e)
    {
        if (!int.TryParse(StaNeighborTxt.Text, out int nb)) nb = 50;
        if (!float.TryParse(StaThreshTxt.Text, out float th)) th = 1.0f;
        RunFilter("统计滤波", () =>
            Filter.staFilter(_inputCloud.PointCloudXYZPointer, nb, th,
                              _outputCloud.PointCloudXYZPointer));
    }

    private void RadiusBtn_Click(object sender, RoutedEventArgs e)
    {
        if (!double.TryParse(RadiusTxt.Text, out double r)) r = 0.1;
        if (!int.TryParse(RadiusMinNeighborTxt.Text, out int mn)) mn = 5;
        RunFilter("半径滤波", () =>
            Filter.radiusFilter(_inputCloud.PointCloudXYZPointer, r, mn,
                                 _outputCloud.PointCloudXYZPointer));
    }

    private void SigmaBtn_Click(object sender, RoutedEventArgs e)
    {
        if (!int.TryParse(SigmaTxt.Text, out int sg)) sg = 3;
        RunFilter("Sigma 滤波", () =>
            Filter.sigamFilter(_inputCloud.PointCloudXYZPointer, sg,
                                _outputCloud.PointCloudXYZPointer));
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Segmentation Tab handlers
    // ═══════════════════════════════════════════════════════════════════════════

    /// <summary>
    /// Run a segmentation algorithm that fills a PointIndices object, then visualise
    /// each cluster with a different colour.
    /// </summary>
    private void RunSegmentation(string name, Action<PointCloudXYZ, PointIndices> segment)
    {
        if (!EnsureInputCloud()) return;
        try
        {
            var indices = new PointIndices();
            segment(_inputCloud, indices);

            // Build per-cluster PointCloudXYZ objects
            var clusters = new List<PointCloudXYZ>();
            for (int i = 0; i < indices.Size; i++)
            {
                var cluster = new PointCloudXYZ();
                Util.copyPcBaseOnIndice(_inputCloud.PointCloudXYZPointer,
                                        indices.GetPointIndice(i),
                                        cluster.PointCloudXYZPointer);
                clusters.Add(cluster);
            }

            ShowClusters(OutputViewport, clusters);

            // Copy the largest cluster into _outputCloud for downstream use
            int maxIdx = 0, maxSize = 0;
            for (int i = 0; i < clusters.Count; i++)
                if (clusters[i].Size > maxSize) { maxSize = clusters[i].Size; maxIdx = i; }

            _outputCloud.Clear();
            for (int i = 0; i < clusters[maxIdx].Size; i++)
                _outputCloud.Push(clusters[maxIdx].GetX(i),
                                   clusters[maxIdx].GetY(i),
                                   clusters[maxIdx].GetZ(i));

            UpdateInfo();
            SetStatus($"{name} 完成 – 找到 {clusters.Count} 个簇，最大簇 {maxSize:N0} 点");
        }
        catch (Exception ex)
        {
            MessageBox.Show($"{name} 失败：{ex.Message}", "错误",
                            MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    private void EucBtn_Click(object sender, RoutedEventArgs e)
    {
        if (!double.TryParse(EucDistTxt.Text, out double dist)) dist = 0.3;
        if (!int.TryParse(EucMinTxt.Text, out int mn)) mn = 10;
        if (!int.TryParse(EucMaxTxt.Text, out int mx)) mx = 1_000_000;

        RunSegmentation("欧式聚类", (cloud, indices) =>
        {
            Segmentation.euclideanCluster(cloud.PointCloudXYZPointer, dist, mn, mx,
                                           indices.PointIndicesPointer);
        });
    }

    private void RgBtn_Click(object sender, RoutedEventArgs e)
    {
        if (!int.TryParse(RgNeighborTxt.Text,  out int nb))    nb    = 50;
        if (!float.TryParse(RgSmoothTxt.Text,  out float sm))  sm    = 3.0f;
        if (!float.TryParse(RgCurveTxt.Text,   out float cv))  cv    = 1.0f;
        if (!int.TryParse(RgMinTxt.Text,        out int minS)) minS  = 50;
        if (!int.TryParse(RgMaxTxt.Text,        out int maxS)) maxS  = 5_000_000;

        RunSegmentation("区域生长", (cloud, indices) =>
        {
            Segmentation.oriGrowRegion(cloud.PointCloudXYZPointer, nb, sm, cv,
                                        minS, maxS, indices.PointIndicesPointer);
        });
    }

    private void ModRgBtn_Click(object sender, RoutedEventArgs e)
    {
        if (!int.TryParse(RgNeighborTxt.Text,  out int nb))    nb    = 50;
        if (!float.TryParse(RgSmoothTxt.Text,  out float sm))  sm    = 3.0f;
        if (!float.TryParse(RgCurveTxt.Text,   out float cv))  cv    = 1.0f;
        if (!int.TryParse(RgMinTxt.Text,        out int minS)) minS  = 50;
        if (!int.TryParse(RgMaxTxt.Text,        out int maxS)) maxS  = 5_000_000;

        if (!EnsureInputCloud()) return;
        try
        {
            _outputCloud.Clear();
            Segmentation.modifiedGrowRegion(_inputCloud.PointCloudXYZPointer, nb, sm, cv,
                                             minS, maxS, _outputCloud.PointCloudXYZPointer);
            RefreshOutput();
            SetStatus($"修正区域生长完成 – 输出 {_outputCloud.Size:N0} 点");
        }
        catch (Exception ex)
        {
            MessageBox.Show($"修正区域生长失败：{ex.Message}", "错误",
                            MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  SampleConsensus / Util Tab handlers
    // ═══════════════════════════════════════════════════════════════════════════

    private void FitPlaneBtn_Click(object sender, RoutedEventArgs e)
    {
        if (!EnsureInputCloud()) return;
        if (!float.TryParse(FitPlaneDistTxt.Text, out float dist)) dist = 0.02f;
        if (!int.TryParse(FitPlaneIteraTxt.Text, out int itera)) itera = 100;

        try
        {
            _planeNormal = new float[4];
            float angle = SampleConsensus.fitPlane(_inputCloud.PointCloudXYZPointer,
                                                    dist, itera, _planeNormal);
            PlaneResultText.Text =
                $"法向量: ({_planeNormal[0]:F4}, {_planeNormal[1]:F4}, " +
                $"{_planeNormal[2]:F4}, {_planeNormal[3]:F4})\n" +
                $"与 Z=0 平面夹角: {angle:F4}°";
            SetStatus($"平面拟合完成 – 夹角 {angle:F4}°");
        }
        catch (Exception ex)
        {
            MessageBox.Show($"平面拟合失败：{ex.Message}", "错误",
                            MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    private void CorrectPlaneBtn_Click(object sender, RoutedEventArgs e)
    {
        if (!EnsureInputCloud()) return;
        if (_planeNormal[0] == 0 && _planeNormal[1] == 0 &&
            _planeNormal[2] == 0 && _planeNormal[3] == 0)
        {
            MessageBox.Show("请先执行「RANSAC 拟合平面」以获取法向量。", "提示");
            return;
        }

        try
        {
            _outputCloud.Clear();
            Util.correctPlane(_inputCloud.PointCloudXYZPointer,
                               _planeNormal,
                               _outputCloud.PointCloudXYZPointer);
            RefreshOutput();
            SetStatus($"平面校正完成 – 输出 {_outputCloud.Size:N0} 点");
        }
        catch (Exception ex)
        {
            MessageBox.Show($"平面校正失败：{ex.Message}", "错误",
                            MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    private void RunoutBtn_Click(object sender, RoutedEventArgs e)
    {
        PointCloudXYZ target = _outputCloud.Size > 0 ? _outputCloud : _inputCloud;
        if (target.Size == 0) { EnsureInputCloud(); return; }

        try
        {
            int[] minMaxIdx = new int[2];
            double runout = Util.calculateRunout(target.PointCloudXYZPointer, minMaxIdx);
            RunoutResultText.Text = $"跳动量：{runout:F6} mm";
            SetStatus($"端面跳动量计算完成：{runout:F6} mm");

            // Highlight the min/max points in the output viewport
            ShowCloud(OutputViewport, target, minMaxIdx[0], minMaxIdx[1]);
        }
        catch (Exception ex)
        {
            MessageBox.Show($"计算跳动量失败：{ex.Message}", "错误",
                            MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }
}
