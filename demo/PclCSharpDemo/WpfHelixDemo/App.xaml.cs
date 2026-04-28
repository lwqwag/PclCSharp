using System;
using System.IO;
using System.Windows;

namespace WpfHelixDemo;

public partial class App : Application
{
    static App()
    {
        var rootBinPath = Path.GetFullPath(Path.Combine(
            AppContext.BaseDirectory,
            "..", "..", "..", "..", "..", "..", "bin"));

        if (!Directory.Exists(rootBinPath))
        {
            return;
        }

        var currentPath = Environment.GetEnvironmentVariable("PATH") ?? string.Empty;
        var hasRootBin = false;

        foreach (var entry in currentPath.Split(Path.PathSeparator, StringSplitOptions.RemoveEmptyEntries))
        {
            if (string.Equals(entry, rootBinPath, StringComparison.OrdinalIgnoreCase))
            {
                hasRootBin = true;
                break;
            }
        }

        if (!hasRootBin)
        {
            var updatedPath = string.IsNullOrEmpty(currentPath)
                ? rootBinPath
                : rootBinPath + Path.PathSeparator + currentPath;
            Environment.SetEnvironmentVariable("PATH", updatedPath);
        }
    }
}
