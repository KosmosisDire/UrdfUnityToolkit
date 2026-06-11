using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

namespace UrdfToolkit.Urdf
{
public class RuntimeUrdf : MonoBehaviour
{
    public string robotDescription = "";
    public bool enableColliders = true;
    public bool enableVisuals = true;
    private List<MonoBehaviour> disabledComponents = new List<MonoBehaviour>();

    void Awake()
    {
        // disable all other components
        foreach (var component in GetComponents<MonoBehaviour>())
        {
            if (component is RuntimeUrdf)
                continue;

            component.enabled = false;
            disabledComponents.Add(component);
        }

        var robot = URDFBuilder.BuildRuntime(robotDescription, gameObject);
        if (!enableColliders)
        {
            robot.DisableColliders();
        }
        if (!enableVisuals)
        {
            robot.DisableVisuals();
        }

        // re-enable all other components
        foreach (var component in disabledComponents)
        {
            component.enabled = true;
        }

        Destroy(this);
    }

}
}
