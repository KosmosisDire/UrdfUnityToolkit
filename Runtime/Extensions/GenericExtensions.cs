using System;
using System.IO;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;

namespace UrdfToolkit.Extensions
{
    public static class GenericExtensions
    {
        public static float ParseFloatOrDefault(this string str, float defaultValue)
        {
            if (float.TryParse(str, out var result))
            {
                return result;
            }
            return defaultValue;
        }

        // add so many spaces to the left of every line in a string
        public static string Indent(this string str, int indent)
        {
            var spaces = new string(' ', indent);
            return spaces + str.Replace("\n", "\n" + spaces);
        }

        public static T DeepCopy<T>(this T other) where T : new()
        {
            if(!typeof(T).IsSerializable && !(typeof(ISerializable).IsAssignableFrom(typeof(T)) ) )
                throw new InvalidOperationException("A serializable Type is required");

            using (MemoryStream ms = new MemoryStream())
            {
                BinaryFormatter formatter = new BinaryFormatter();
                formatter.Serialize(ms, other);
                ms.Position = 0;
                return (T)formatter.Deserialize(ms);
            }
        }
    }
}