using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Xml;
using UnityEngine;

namespace UrdfToolkit.Xacro
{
    /// <summary>A parsed macro parameter from a <c>params="..."</c> attribute.</summary>
    internal struct MacroParam
    {
        public string Name;     // bare name, without '*'/'**' or ':=default'
        public int Stars;       // 0 = text param, 1 = '*block', 2 = '**block'
        public bool HasDefault;
        public string Default;
    }

    /// <summary>A <c>&lt;xacro:macro&gt;</c> definition: its parameters and a clonable body.</summary>
    internal sealed class XacroMacro
    {
        public string Name;
        public List<MacroParam> Params;
        public List<XmlNode> Body;
    }

    /// <summary>
    /// The set of names visible at a point in the tree: text properties, captured blocks
    /// (for <c>*</c>/<c>**</c> macro params and <c>insert_block</c>), and macro definitions.
    /// A child scope is a shallow copy so definitions inside a macro/conditional don't leak out.
    /// </summary>
    internal sealed class Scope
    {
        public Dictionary<string, string> Properties;
        public Dictionary<string, List<XmlNode>> Blocks;
        public Dictionary<string, XacroMacro> Macros;

        public static Scope Root() => new Scope
        {
            Properties = new Dictionary<string, string>(),
            Blocks = new Dictionary<string, List<XmlNode>>(),
            Macros = new Dictionary<string, XacroMacro>(),
        };

        public Scope Child() => new Scope
        {
            Properties = new Dictionary<string, string>(Properties),
            Blocks = new Dictionary<string, List<XmlNode>>(Blocks),
            Macros = new Dictionary<string, XacroMacro>(Macros),
        };
    }

    /// <summary>
    /// Expands a xacro document in place: a single in-order recursive walk that resolves
    /// includes, args, properties, conditionals, macros and <c>${...}</c> expressions. This mirrors
    /// xacro's own processing model, where the symbol table evolves as the tree is walked (so a
    /// property defined partway through a macro body is visible to the rest of that body).
    /// </summary>
    public sealed class XacroProcessor
    {
        private readonly XmlDocument doc;
        private readonly string rootPath;
        private readonly XacroExpressionEvaluator evaluator;
        private readonly Dictionary<string, string> args = new Dictionary<string, string>();
        private readonly List<string> missingIncludes = new List<string>();
        private readonly List<string> macroStack = new List<string>();
        private string currentFile;

        // Lazily-built ROS package index, shared with URDF package:// resolution.
        private RosPackageIndex packages;

        public XacroProcessor(XmlDocument doc, string sourceFile, string rootPath, XacroExpressionEvaluator evaluator)
        {
            this.doc = doc;
            this.currentFile = sourceFile;
            this.rootPath = rootPath;
            this.evaluator = evaluator;
        }

        public void Process()
        {
            ProcessChildren(doc.DocumentElement, Scope.Root());

            // If processing finished but some includes never resolved, surface that clearly rather
            // than leaving a silently-incomplete robot. (A missing include that *was* needed will
            // usually have already failed earlier, with the same explanation attached.)
            if (missingIncludes.Count > 0)
                throw new XacroException(
                    $"{missingIncludes.Count} xacro include(s) could not be found:{MissingIncludesBlock()}\n" +
                    "Add the missing package(s) or file(s) and re-import.");
        }

        // --- tree walk ---

        private void ProcessChildren(XmlElement parent, Scope scope)
        {
            // Snapshot first: processing rewrites the child list (inlining, removals).
            foreach (var child in parent.ChildNodes.Cast<XmlNode>().ToList())
                ProcessNode(child, scope);
        }

        private void ProcessNode(XmlNode node, Scope scope)
        {
            if (node is XmlElement el)
            {
                if (el.Prefix == "xacro")
                    ProcessXacroElement(el, scope);
                else
                {
                    EvalAttributes(el, scope);
                    ProcessChildren(el, scope);
                }
            }
            else if (node is XmlText || node is XmlCDataSection || node is XmlSignificantWhitespace)
            {
                if (node.Value != null && node.Value.IndexOf('$') >= 0)
                    node.Value = Eval(node.Value, scope);
            }
        }

        private void ProcessXacroElement(XmlElement el, Scope scope)
        {
            switch (el.LocalName)
            {
                case "property":
                    DefineProperty(el, scope);
                    Remove(el);
                    break;
                case "arg":
                    DefineArg(el, scope);
                    Remove(el);
                    break;
                case "macro":
                    RegisterMacro(el, scope);
                    Remove(el);
                    break;
                case "include":
                    ProcessInclude(el, scope);
                    break;
                case "if":
                    ProcessConditional(el, scope, negate: false);
                    break;
                case "unless":
                    ProcessConditional(el, scope, negate: true);
                    break;
                case "insert_block":
                case "insert-block":
                    ProcessInsertBlock(el, scope);
                    break;
                default:
                    if (scope.Macros.TryGetValue(el.LocalName, out var macro))
                        ExpandMacro(el, macro, scope);
                    else
                        throw new XacroException(
                            $"Undefined macro or unsupported directive '<xacro:{el.LocalName}>' in {FileLabel()}." +
                            (missingIncludes.Count > 0
                                ? $" Its definition is probably in an include that could not be found:{MissingIncludesBlock()}"
                                : " If it is a macro, its <xacro:include> may be missing or misspelled."));
                    break;
            }
        }

        // --- directives ---

        private void DefineProperty(XmlElement el, Scope scope)
        {
            string name = el.GetAttribute("name");
            if (name.StartsWith("*"))
            {
                scope.Blocks[name.TrimStart('*')] = CloneChildren(el);
                return;
            }

            // Real xacro uses value="..."; some descriptions sloppily use default="..." — accept both.
            string raw = el.HasAttribute("value") ? el.GetAttribute("value")
                : el.HasAttribute("default") ? el.GetAttribute("default")
                : null;

            if (raw == null && el.ChildNodes.Count > 0)
            {
                // A property whose body is markup rather than a scalar value.
                scope.Blocks[name] = CloneChildren(el);
                return;
            }

            scope.Properties[name] = Eval(raw ?? string.Empty, scope);
        }

        private void DefineArg(XmlElement el, Scope scope)
        {
            string name = el.GetAttribute("name");
            if (!args.ContainsKey(name))
                args[name] = Eval(el.GetAttribute("default"), scope);
        }

        private void RegisterMacro(XmlElement el, Scope scope)
        {
            scope.Macros[el.GetAttribute("name")] = new XacroMacro
            {
                Name = el.GetAttribute("name"),
                Params = ParseParams(el.GetAttribute("params")),
                Body = CloneChildren(el),
            };
        }

        private void ProcessConditional(XmlElement el, Scope scope, bool negate)
        {
            string cond = el.GetAttribute("value");
            bool keep = EvalBool(cond, scope);
            if (negate) keep = !keep;

            if (keep)
            {
                // Inline the children into the parent and process them in the SAME scope, so a
                // property defined inside the block is visible to following siblings (as in xacro).
                foreach (var child in el.ChildNodes.Cast<XmlNode>().ToList())
                {
                    el.ParentNode.InsertBefore(child, el);
                    ProcessNode(child, scope);
                }
            }
            Remove(el);
        }

        private void ProcessInsertBlock(XmlElement el, Scope scope)
        {
            string name = el.GetAttribute("name").TrimStart('*');
            if (scope.Blocks.TryGetValue(name, out var nodes))
            {
                foreach (var n in nodes)
                {
                    var imported = el.OwnerDocument.ImportNode(n, true);
                    el.ParentNode.InsertBefore(imported, el);
                    ProcessNode(imported, scope);
                }
            }
            else
            {
                Warn($"insert_block referenced unknown block '{name}' in {FileLabel()}");
            }
            Remove(el);
        }

        private void ExpandMacro(XmlElement call, XacroMacro macro, Scope scope)
        {
            // Guard against non-terminating recursion (which would otherwise be an uncatchable
            // StackOverflow) and report the chain so the offending macro is obvious.
            if (macroStack.Count(n => n == macro.Name) >= 64 || macroStack.Count > 512)
            {
                var tail = macroStack.Skip(Math.Max(0, macroStack.Count - 8));
                throw new XacroException(
                    $"Macro expansion went too deep at '<xacro:{macro.Name}>' in {FileLabel()} " +
                    $"(…{string.Join(" -> ", tail)} -> {macro.Name}). " +
                    "This usually means a recursive macro never reached its stop condition.");
            }

            macroStack.Add(macro.Name);
            try
            {
                ExpandMacroBody(call, macro, scope);
            }
            finally
            {
                macroStack.RemoveAt(macroStack.Count - 1);
            }
        }

        private void ExpandMacroBody(XmlElement call, XacroMacro macro, Scope scope)
        {
            var inner = scope.Child();
            var callElements = call.ChildNodes.Cast<XmlNode>().OfType<XmlElement>().ToList();
            int starCursor = 0;

            foreach (var p in macro.Params)
            {
                if (p.Stars == 2)
                {
                    var match = callElements.FirstOrDefault(c => c.LocalName == p.Name);
                    inner.Blocks[p.Name] = match != null
                        ? ExpandBlockArgument(match.ChildNodes.Cast<XmlNode>().ToList(), scope)
                        : new List<XmlNode>();
                }
                else if (p.Stars == 1)
                {
                    var block = starCursor < callElements.Count ? callElements[starCursor++] : null;
                    inner.Blocks[p.Name] = block != null
                        ? ExpandBlockArgument(new List<XmlNode> { block }, scope)
                        : new List<XmlNode>();
                }
                else if (call.HasAttribute(p.Name))
                {
                    // Macro arguments are evaluated in the CALLER's scope, then bound locally.
                    inner.Properties[p.Name] = Eval(call.GetAttribute(p.Name), scope);
                }
                else if (p.HasDefault)
                {
                    inner.Properties[p.Name] = Eval(p.Default, inner);
                }
                else
                {
                    throw new XacroException(
                        $"Macro '{macro.Name}' is missing required parameter '{p.Name}' in {FileLabel()}.");
                }
            }

            // Expand the body in a detached container, then splice the results in front of the call.
            var temp = doc.CreateElement("xacro_temp");
            foreach (var n in macro.Body)
                temp.AppendChild(n.CloneNode(true));
            ProcessChildren(temp, inner);

            foreach (var n in temp.ChildNodes.Cast<XmlNode>().ToList())
                call.ParentNode.InsertBefore(n, call);
            Remove(call);
        }

        /// <summary>
        /// Block arguments are caller-provided markup, so they are expanded in the CALLER's scope at
        /// the call site (resolving any nested <c>insert_block</c> / <c>${...}</c>). This is also what
        /// stops a forwarded block — <c>&lt;xacro:foo&gt;&lt;xacro:insert_block name="x"/&gt;&lt;/xacro:foo&gt;</c> —
        /// from re-resolving to itself inside the callee and recursing forever.
        /// </summary>
        private List<XmlNode> ExpandBlockArgument(List<XmlNode> sourceNodes, Scope callerScope)
        {
            var temp = doc.CreateElement("xacro_temp");
            foreach (var n in sourceNodes)
                temp.AppendChild(n.CloneNode(true));
            ProcessChildren(temp, callerScope);
            return temp.ChildNodes.Cast<XmlNode>().Select(n => n.CloneNode(true)).ToList();
        }

        // --- includes ---

        private void ProcessInclude(XmlElement el, Scope scope)
        {
            string raw = el.GetAttribute("filename");
            string filename = ResolveIncludePath(raw, scope);
            if (!Path.IsPathRooted(filename))
                filename = Path.Combine(rootPath, filename);

            if (File.Exists(filename))
            {
                var included = new XmlDocument();
                included.Load(filename);

                string previousFile = currentFile;
                currentFile = filename;
                foreach (XmlNode child in included.DocumentElement.ChildNodes)
                {
                    var imported = doc.ImportNode(child, true);
                    el.ParentNode.InsertBefore(imported, el);
                    ProcessNode(imported, scope);
                }
                currentFile = previousFile;
            }
            else
            {
                RecordMissingInclude(raw, filename);
            }
            Remove(el);
        }

        private void RecordMissingInclude(string raw, string resolved)
        {
            // If a $(find pkg) token is still present, the package itself couldn't be located.
            var find = Regex.Match(resolved, @"\$\(find\s+([^)]*)\)");
            string description = find.Success
                ? $"ROS package '{find.Groups[1].Value.Trim()}' not found (needed for \"{raw}\", referenced by {FileLabel()})"
                : $"\"{resolved}\" not found (referenced by {FileLabel()})";

            if (!missingIncludes.Contains(description))
            {
                missingIncludes.Add(description);
                Warn("missing include — " + description);
            }
        }

        private string MissingIncludesBlock()
        {
            var sb = new StringBuilder();
            foreach (var item in missingIncludes)
                sb.Append("\n  - ").Append(item);
            return sb.ToString();
        }

        /// <summary>
        /// Resolves <c>$(find pkg)</c> (in C#, by locating the package folder) and <c>$(arg ...)</c>/<c>${...}</c>
        /// (in Python). Alternates the two until the path stops changing, since an arg can expand
        /// into another <c>$(find ...)</c>.
        /// </summary>
        private string ResolveIncludePath(string raw, Scope scope)
        {
            string current = raw;
            string previous = null;
            for (int i = 0; i < 5 && current != previous && (current.Contains("$(") || current.Contains("${")); i++)
            {
                previous = current;
                current = ResolveFindTokens(current, scope);
                current = Eval(current, scope);
            }
            return current;
        }

        private string ResolveFindTokens(string text, Scope scope)
        {
            return Regex.Replace(text, @"\$\(find\s+([^)]*)\)", match =>
            {
                string package = Eval(match.Groups[1].Value, scope).Trim();
                string dir = FindPackageDirectory(package);
                // If not found, normalize to the resolved package name so the not-found message is accurate.
                return dir ?? $"$(find {package})";
            });
        }

        /// <summary>
        /// Resolves a ROS package name to a directory, the way <c>$(find pkg)</c> does. We have no ROS
        /// package index on disk, so we build one from the <c>package.xml</c> files found under the Unity
        /// project's <c>Assets</c> folder (a package's authoritative name is its <c>&lt;name&gt;</c> element,
        /// not its folder name). A folder-name match is used as a fallback for partial/copied packages
        /// that ship without a package.xml.
        /// </summary>
        private string FindPackageDirectory(string package)
        {
            if (packages == null)
            {
                packages = new RosPackageIndex(rootPath);
                Debug.Log($"[xacro] resolving $(find ...) under '{packages.SearchRoot}' — found {packages.Count} package(s)");
            }
            return packages.FindPackage(package);
        }

        // --- expression evaluation ---

        private void EvalAttributes(XmlElement el, Scope scope)
        {
            foreach (XmlAttribute attr in el.Attributes)
                if (attr.Value.IndexOf('$') >= 0)
                    attr.Value = Eval(attr.Value, scope);
        }

        private string Eval(string text, Scope scope)
        {
            try
            {
                return evaluator.Eval(text, scope.Properties, args);
            }
            catch (XacroException ex)
            {
                throw Located(ex, text);
            }
        }

        private bool EvalBool(string text, Scope scope)
        {
            try
            {
                return evaluator.EvalBool(text, scope.Properties, args);
            }
            catch (XacroException ex)
            {
                throw Located(ex, text);
            }
        }

        private XacroException Located(XacroException inner, string text)
        {
            // A NameError almost always means a property/macro from a missing include is absent.
            // Reframe it as the missing include rather than a raw Python error when we know of one.
            var undefined = Regex.Match(inner.Message, @"name '([^']+)' is not defined");
            if (undefined.Success && missingIncludes.Count > 0)
            {
                return new XacroException(
                    $"Cannot expand \"{text.Trim()}\" in {FileLabel()}: '{undefined.Groups[1].Value}' is undefined " +
                    $"because {missingIncludes.Count} include(s) could not be found:{MissingIncludesBlock()}\n" +
                    "Add the missing package(s) or file(s) and re-import.", inner);
            }

            return new XacroException($"Failed to evaluate \"{text.Trim()}\" in {FileLabel()}: {inner.Message}", inner);
        }

        // --- helpers ---

        private static List<MacroParam> ParseParams(string spec)
        {
            var result = new List<MacroParam>();
            if (string.IsNullOrWhiteSpace(spec)) return result;

            foreach (var token in spec.Split(new[] { ' ', '\t', '\n', '\r' }, System.StringSplitOptions.RemoveEmptyEntries))
            {
                var param = new MacroParam();
                string name = token;
                if (name.StartsWith("**")) { param.Stars = 2; name = name.Substring(2); }
                else if (name.StartsWith("*")) { param.Stars = 1; name = name.Substring(1); }

                int defIdx = name.IndexOf(":=");
                if (defIdx >= 0)
                {
                    param.HasDefault = true;
                    param.Default = name.Substring(defIdx + 2);
                    name = name.Substring(0, defIdx);
                }
                param.Name = name;
                result.Add(param);
            }
            return result;
        }

        private static List<XmlNode> CloneChildren(XmlElement el)
        {
            return el.ChildNodes.Cast<XmlNode>().Select(n => n.CloneNode(true)).ToList();
        }

        private static void Remove(XmlElement el)
        {
            el.ParentNode?.RemoveChild(el);
        }

        private string FileLabel()
        {
            return Path.GetFileName(currentFile);
        }

        private static void Warn(string message)
        {
            Debug.LogWarning("[xacro] " + message);
        }
    }
}
