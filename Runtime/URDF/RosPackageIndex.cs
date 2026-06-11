using System;
using System.Collections.Generic;
using System.IO;
using System.Xml;

namespace UrdfToolkit
{
    /// <summary>
    /// Resolves ROS package names to directories the way ROS tooling does: by indexing the
    /// <c>package.xml</c> files found under the Unity project's <c>Assets</c> folder (a package's
    /// authoritative name is its <c>&lt;name&gt;</c> element, not its folder name). Shared by xacro
    /// <c>$(find pkg)</c> resolution and URDF <c>package://</c> mesh resolution.
    /// </summary>
    public sealed class RosPackageIndex
    {
        private static readonly string[] SkipDirs =
            { "Library", "Temp", "Logs", "obj", "bin", "node_modules", "Build", "Builds" };

        private readonly Dictionary<string, string> byName = new Dictionary<string, string>();

        /// <summary>The directory the index was built from.</summary>
        public string SearchRoot { get; }

        /// <summary>Number of packages discovered.</summary>
        public int Count => byName.Count;

        public RosPackageIndex(string startPath)
        {
            SearchRoot = DetermineSearchRoot(startPath);
            if (Directory.Exists(SearchRoot))
                Scan(SearchRoot, 0);
        }

        /// <summary>Directory for a package name, or null. Falls back to a folder-name match.</summary>
        public string FindPackage(string package)
        {
            if (string.IsNullOrEmpty(package)) return null;
            if (byName.TryGetValue(package, out var dir)) return dir;

            try
            {
                var matches = Directory.GetDirectories(SearchRoot, package, SearchOption.AllDirectories);
                if (matches.Length > 0) return matches[0];
            }
            catch { /* traversal errors (permissions, reparse points) -> not found */ }

            return null;
        }

        /// <summary>
        /// Resolves a <c>package://pkg/relative/path</c> URL to an absolute file path, or null if the
        /// package can't be found. A value that isn't a package:// URL is returned unchanged.
        /// </summary>
        public string ResolvePackageUrl(string url)
        {
            if (string.IsNullOrEmpty(url)) return null;

            const string scheme = "package://";
            if (!url.StartsWith(scheme, StringComparison.Ordinal)) return url;

            string rest = url.Substring(scheme.Length);
            int slash = rest.IndexOfAny(new[] { '/', '\\' });
            if (slash < 0) return null;

            string package = rest.Substring(0, slash);
            string relative = rest.Substring(slash + 1);
            string dir = FindPackage(package);
            return dir != null ? Path.Combine(dir, relative) : null;
        }

        private static string DetermineSearchRoot(string startPath)
        {
            DirectoryInfo dir;
            try { dir = new DirectoryInfo(startPath); }
            catch { return startPath; }

            for (var d = dir; d != null; d = d.Parent)
            {
                if (string.Equals(d.Name, "Assets", StringComparison.OrdinalIgnoreCase))
                    return d.FullName;
            }
            // Not under an Assets folder (e.g. standalone tests): search the start path's parent.
            return dir.Parent?.FullName ?? dir.FullName;
        }

        private void Scan(string dir, int depth)
        {
            if (depth > 32) return;

            string packageXml = Path.Combine(dir, "package.xml");
            if (File.Exists(packageXml))
            {
                string name = ReadPackageName(packageXml);
                if (!string.IsNullOrEmpty(name) && !byName.ContainsKey(name))
                    byName[name] = dir;
            }

            string[] subdirs;
            try { subdirs = Directory.GetDirectories(dir); }
            catch { return; }

            foreach (var sub in subdirs)
            {
                string leaf = Path.GetFileName(sub);
                if (leaf.StartsWith(".", StringComparison.Ordinal) || Array.IndexOf(SkipDirs, leaf) >= 0)
                    continue;
                Scan(sub, depth + 1);
            }
        }

        private static string ReadPackageName(string packageXmlPath)
        {
            try
            {
                var doc = new XmlDocument();
                doc.Load(packageXmlPath);
                return doc.DocumentElement?.SelectSingleNode("name")?.InnerText.Trim();
            }
            catch { return null; }
        }
    }
}
