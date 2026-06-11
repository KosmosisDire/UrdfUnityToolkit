using System.Xml;
using UnityEngine;

namespace UrdfToolkit.Extensions
{
    public static class XMLExtennsions
    {
        public static double GetDouble(this XmlNode node, string name, double defaultValue = 0)
        {
            var attribute = node?.Attributes?[name];
            if (attribute == null)
            {
                return defaultValue;
            }
            return double.Parse(attribute.Value);
        }

        public static string GetString(this XmlNode node, string name, string defaultValue = "")
        {
            var attribute = node?.Attributes?[name];
            if (attribute == null)
            {
                return defaultValue;
            }
            return attribute.Value;
        }

        public static bool GetBool(this XmlNode node, string name, bool defaultValue = false)
        {
            var attribute = node?.Attributes?[name];
            if (attribute == null)
            {
                return defaultValue;
            }
            return bool.Parse(attribute.Value);
        }

        public static int GetInt(this XmlNode node, string name, int defaultValue = 0)
        {
            var attribute = node?.Attributes?[name];
            if (attribute == null)
            {
                return defaultValue;
            }
            return int.Parse(attribute.Value);
        }

        public static float GetFloat(this XmlNode node, string name, float defaultValue = 0)
        {
            var attribute = node?.Attributes?[name];
            if (attribute == null)
            {
                return defaultValue;
            }
            return float.Parse(attribute.Value);
        }

        public static Vector3 GetVector3(this XmlNode node, string name, Vector3 defaultValue = default)
        {
            var attribute = node?.Attributes?[name];
            if (attribute == null)
            {
                return defaultValue;
            }
            var values = attribute.Value.Split(' ');
            if (values.Length != 3)
            {
                return defaultValue;
            }
            return new Vector3(
                float.Parse(values[0]),
                float.Parse(values[1]),
                float.Parse(values[2])
            );
        }
    }
}