using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Threading;

namespace UrdfToolkit.Xacro
{
    /// <summary>
    /// A xacro processing/evaluation failure carrying a message intended for the user.
    /// </summary>
    public class XacroException : Exception
    {
        public XacroException(string message) : base(message) { }
        public XacroException(string message, Exception inner) : base(message, inner) { }
    }

    /// <summary>
    /// Evaluates xacro <c>${...}</c> expressions and <c>xacro:if</c>/<c>unless</c> conditions.
    /// Xacro expressions are arbitrary Python, so rather than reimplement a Python evaluator we
    /// delegate to a system Python 3 interpreter running a small helper script. Python is required.
    /// </summary>
    public sealed class XacroExpressionEvaluator : IDisposable
    {
        private static readonly string[] PythonCandidates = { "python3", "python", "py" };
        private static readonly UTF8Encoding Utf8NoBom = new UTF8Encoding(false);

        private Process process;
        private StreamWriter stdin;
        private StreamReader stdout;
        private string scriptPath;
        private bool started;
        private readonly StringBuilder stderr = new StringBuilder();
        private readonly object gate = new object();

        /// <summary>The interpreter command that was successfully launched (for diagnostics).</summary>
        public string PythonCommand { get; private set; }

        public XacroExpressionEvaluator()
        {
            // Cheap by design: Python is not launched until the first expression actually needs it
            // (see EnsureStarted). A xacro with no ${...} expressions never requires Python at all.
        }

        /// <summary>Evaluates text containing <c>${...}</c> / <c>$(arg ...)</c> and returns the substituted string.</summary>
        public string Eval(string text, IReadOnlyDictionary<string, string> vars, IReadOnlyDictionary<string, string> args)
        {
            // No '$' means nothing to substitute — it evaluates to itself, so we needn't touch Python.
            if (string.IsNullOrEmpty(text) || text.IndexOf('$') < 0)
                return text;

            EnsureStarted();
            return Request("eval", text, vars, args);
        }

        /// <summary>Evaluates text as a boolean condition (xacro:if / xacro:unless semantics).</summary>
        public bool EvalBool(string text, IReadOnlyDictionary<string, string> vars, IReadOnlyDictionary<string, string> args)
        {
            // Plain boolean / integer literals don't need Python (mirrors xacro's get_boolean_value).
            string trimmed = text == null ? string.Empty : text.Trim();
            if (trimmed.IndexOf('$') < 0)
            {
                if (trimmed == "true" || trimmed == "True") return true;
                if (trimmed == "false" || trimmed == "False") return false;
                if (int.TryParse(trimmed, out int n)) return n != 0;
                // Anything else falls through so Python yields the same error xacro would.
            }

            EnsureStarted();
            return Request("cond", text, vars, args) == "1";
        }

        /// <summary>Launches the Python helper on first use; throws a clear error if Python is unavailable.</summary>
        private void EnsureStarted()
        {
            if (started) return;
            lock (gate)
            {
                if (started) return;

                scriptPath = WriteHelperScript();
                Exception last = null;
                foreach (var exe in PythonCandidates)
                {
                    Process candidate = null;
                    try
                    {
                        candidate = StartProcess(exe, scriptPath);
                        // Wrap the base streams in explicit UTF-8 so this works regardless of the
                        // project's API-compatibility level (StandardInputEncoding is not available
                        // on the .NET Framework profile). The helper forces UTF-8 on its side too.
                        var input = new StreamWriter(candidate.StandardInput.BaseStream, Utf8NoBom) { AutoFlush = false };
                        var output = new StreamReader(candidate.StandardOutput.BaseStream, Utf8NoBom);
                        DrainStderr(candidate);

                        // Confirm the interpreter actually runs the helper and speaks the protocol.
                        if (Exchange(input, output, "eval", "__ping__", null, null) == "__ping__")
                        {
                            process = candidate;
                            stdin = input;
                            stdout = output;
                            PythonCommand = exe;
                            started = true;
                            return;
                        }
                    }
                    catch (Exception e)
                    {
                        last = e;
                    }

                    // This candidate did not work out; clean up and try the next.
                    try { candidate?.Kill(); } catch { /* ignore */ }
                    try { candidate?.Dispose(); } catch { /* ignore */ }
                }

                throw new XacroException(
                    "Xacro needs Python 3 to evaluate ${...} expressions, but none of 'python3', 'python', or 'py' " +
                    "could be started from your PATH. Install Python 3 (https://www.python.org/downloads/) and make " +
                    "sure it is on your PATH, then re-import." +
                    (last != null ? " (last error: " + last.Message + ")" : string.Empty));
            }
        }

        private string Request(string op, string text, IReadOnlyDictionary<string, string> vars, IReadOnlyDictionary<string, string> args)
        {
            lock (gate)
            {
                if (process == null || process.HasExited)
                    throw new XacroException("The Python expression evaluator is not running." + StderrTail());
                return Exchange(stdin, stdout, op, text, vars, args);
            }
        }

        private string Exchange(StreamWriter input, StreamReader output, string op, string text, IReadOnlyDictionary<string, string> vars, IReadOnlyDictionary<string, string> args)
        {
            input.Write(BuildRequestJson(op, text, vars, args));
            input.Write('\n');
            input.Flush();

            string line = output.ReadLine();
            if (line == null)
                throw new XacroException("The Python expression evaluator stopped responding." + StderrTail());

            int sp = line.IndexOf(' ');
            string tag = sp < 0 ? line : line.Substring(0, sp);
            string payload = sp < 0 ? string.Empty : line.Substring(sp + 1);
            switch (tag)
            {
                case "V": return DecodeBase64(payload);
                case "B": return payload; // "1" or "0"
                case "E": throw new XacroException(DecodeBase64(payload));
                default: throw new XacroException("Malformed response from Python evaluator: " + line);
            }
        }

        // --- request encoding (hand-rolled so we need no JSON library on the C# side) ---

        private static string BuildRequestJson(string op, string text, IReadOnlyDictionary<string, string> vars, IReadOnlyDictionary<string, string> args)
        {
            var sb = new StringBuilder(64);
            sb.Append('{');
            sb.Append("\"op\":");
            AppendJsonString(sb, op);
            sb.Append(",\"text\":");
            AppendJsonString(sb, text ?? string.Empty);
            sb.Append(",\"vars\":");
            AppendJsonObject(sb, vars);
            sb.Append(",\"args\":");
            AppendJsonObject(sb, args);
            sb.Append('}');
            return sb.ToString();
        }

        private static void AppendJsonObject(StringBuilder sb, IReadOnlyDictionary<string, string> map)
        {
            sb.Append('{');
            if (map != null)
            {
                bool first = true;
                foreach (var kv in map)
                {
                    if (!first) sb.Append(',');
                    first = false;
                    AppendJsonString(sb, kv.Key);
                    sb.Append(':');
                    AppendJsonString(sb, kv.Value ?? string.Empty);
                }
            }
            sb.Append('}');
        }

        private static void AppendJsonString(StringBuilder sb, string s)
        {
            sb.Append('"');
            foreach (char c in s)
            {
                switch (c)
                {
                    case '"': sb.Append("\\\""); break;
                    case '\\': sb.Append("\\\\"); break;
                    case '\b': sb.Append("\\b"); break;
                    case '\f': sb.Append("\\f"); break;
                    case '\n': sb.Append("\\n"); break;
                    case '\r': sb.Append("\\r"); break;
                    case '\t': sb.Append("\\t"); break;
                    default:
                        if (c < 0x20)
                            sb.Append("\\u").Append(((int)c).ToString("x4"));
                        else
                            sb.Append(c);
                        break;
                }
            }
            sb.Append('"');
        }

        private static string DecodeBase64(string payload)
        {
            if (string.IsNullOrEmpty(payload)) return string.Empty;
            return Encoding.UTF8.GetString(Convert.FromBase64String(payload));
        }

        // --- process management ---

        private static Process StartProcess(string exe, string script)
        {
            var psi = new ProcessStartInfo
            {
                FileName = exe,
                // Quote the path (temp dir may contain spaces). ArgumentList isn't on every profile.
                Arguments = "\"" + script + "\"",
                UseShellExecute = false,
                RedirectStandardInput = true,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                CreateNoWindow = true,
            };

            var p = new Process { StartInfo = psi };
            p.Start();
            return p;
        }

        private void DrainStderr(Process p)
        {
            var t = new Thread(() =>
            {
                try
                {
                    string l;
                    while ((l = p.StandardError.ReadLine()) != null)
                    {
                        lock (stderr) { stderr.AppendLine(l); }
                    }
                }
                catch { /* process gone */ }
            })
            { IsBackground = true, Name = "xacro-python-stderr" };
            t.Start();
        }

        private string StderrTail()
        {
            string err;
            lock (stderr) { err = stderr.ToString(); }
            return string.IsNullOrWhiteSpace(err) ? string.Empty : " Python stderr:\n" + err.Trim();
        }

        private string WriteHelperScript()
        {
            var path = Path.Combine(Path.GetTempPath(), "urdftoolkit_xacro_eval.py");
            File.WriteAllText(path, HelperScript, new UTF8Encoding(false));
            return path;
        }

        public void Dispose()
        {
            try { stdin?.Close(); } catch { /* ignore */ }
            try
            {
                if (process != null && !process.WaitForExit(1000))
                    process.Kill();
            }
            catch { /* ignore */ }
            try { process?.Dispose(); } catch { /* ignore */ }
            try { if (scriptPath != null && File.Exists(scriptPath)) File.Delete(scriptPath); } catch { /* ignore */ }
        }

        // The helper deliberately uses only single-quoted Python strings so it can be embedded
        // here as a verbatim string with no escaping. It mirrors the parts of xacro's evaluator
        // that real robot descriptions rely on (eval_literal coercion, the global symbol set,
        // ${...}/$(arg ...) substitution and get_boolean_value).
        private const string HelperScript = @"import sys, json, math, builtins, base64


def _build_globals():
    g = {}
    # Expose every public math symbol directly: pi, sin, cos, radians, degrees, ...
    for k, v in math.__dict__.items():
        if not k.startswith('_'):
            g[k] = v
    g['math'] = math
    # Builtins xacro makes available inside the expression namespace.
    for name in ('list', 'dict', 'map', 'len', 'str', 'float', 'int', 'bool',
                 'min', 'max', 'round', 'abs', 'range', 'sorted', 'zip',
                 'enumerate', 'filter', 'sum', 'tuple', 'set', 'all', 'any'):
        if hasattr(builtins, name):
            g[name] = getattr(builtins, name)
    g['True'] = True
    g['False'] = False
    g['None'] = None
    return g


GLOBALS = _build_globals()


def get_boolean_value(value):
    # Mirrors xacro.get_boolean_value.
    if isinstance(value, str):
        if value == 'true' or value == 'True':
            return True
        if value == 'false' or value == 'False':
            return False
        return bool(int(value))
    return bool(value)


def eval_literal(value):
    # Mirrors xacro Table._eval_literal: type-coerce property values for the namespace.
    if isinstance(value, str):
        if len(value) >= 2 and value[0] == chr(39) and value[-1] == chr(39):
            return value[1:-1]
        if '_' in value:
            return value
        for f in (int, float, get_boolean_value):
            try:
                return f(value)
            except Exception:
                pass
    return value


def make_symbols(variables):
    return {k: eval_literal(v) for k, v in (variables or {}).items()}


def stringify(value):
    if isinstance(value, float):
        return repr(value)
    return str(value)


def resolve_sub(token, args):
    # token looks like $(arg name) or $(find pkg) etc.
    inner = token[2:-1].strip()
    parts = inner.split(None, 1)
    cmd = parts[0] if parts else ''
    rest = parts[1].strip() if len(parts) > 1 else ''
    if cmd == 'arg':
        if rest in args:
            return str(args[rest])
        raise NameError('undefined arg ' + repr(rest) + ' (referenced via $(arg ' + rest + '))')
    # Leave $(find ...) and other substitutions untouched; include-path resolution handles them.
    return token


def split_text(text):
    parts = []
    buf = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c == '$' and i + 1 < n and text[i + 1] == '$':
            buf.append('$'); i += 2; continue
        if c == '$' and i + 1 < n and text[i + 1] == '{':
            j = text.find('}', i + 2)
            if j != -1:
                if buf: parts.append(('lit', ''.join(buf))); buf = []
                parts.append(('expr', text[i + 2:j])); i = j + 1; continue
        if c == '$' and i + 1 < n and text[i + 1] == '(':
            j = text.find(')', i + 2)
            if j != -1:
                if buf: parts.append(('lit', ''.join(buf))); buf = []
                parts.append(('sub', text[i:j + 1])); i = j + 1; continue
        buf.append(c); i += 1
    if buf: parts.append(('lit', ''.join(buf)))
    return parts


def evaluate(text, symbols, args):
    # Resolve $(...) substitutions into literals first.
    parts = [('lit', resolve_sub(s, args)) if k == 'sub' else (k, s)
             for k, s in split_text(text)]
    # A string that is exactly one ${expr} keeps the expression native type.
    if len(parts) == 1 and parts[0][0] == 'expr':
        return eval(parts[0][1], dict(GLOBALS), symbols)
    out = []
    for kind, s in parts:
        out.append(stringify(eval(s, dict(GLOBALS), symbols)) if kind == 'expr' else s)
    return ''.join(out)


def b64(s):
    return base64.b64encode(s.encode('utf-8')).decode('ascii')


def handle(req):
    # Returns a single response line. Tags: V <b64 value>, B <0|1>, E <b64 error>.
    op = req.get('op')
    text = req.get('text', '')
    symbols = make_symbols(req.get('vars'))
    args = req.get('args') or {}
    if op == 'eval':
        return 'V ' + b64(stringify(evaluate(text, symbols, args)))
    if op == 'cond':
        return 'B ' + ('1' if get_boolean_value(evaluate(text, symbols, args)) else '0')
    return 'E ' + b64('unknown op ' + repr(op))


def main():
    # Force UTF-8 so non-ASCII in expressions survives the Windows console codepage.
    try:
        sys.stdin.reconfigure(encoding='utf-8')
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
            resp = handle(req)
        except Exception as e:
            resp = 'E ' + b64(type(e).__name__ + ': ' + str(e))
        sys.stdout.write(resp + chr(10))
        sys.stdout.flush()


if __name__ == '__main__':
    main()
";
    }
}
