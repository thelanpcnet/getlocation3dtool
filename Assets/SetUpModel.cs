using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SetUpModel : MonoBehaviour
{
    public Transform parent;
    public int n = 0;

    bool des = false;

    // Start is called before the first frame update
    void Start()
    {
        n = parent.childCount;
    }

    private void Update()
    {
        foreach (Transform child in parent)
        {
            SetNewParent(child);
        }

        //Press E to destroy empty project
        if (Input.GetKeyDown(KeyCode.E))
        {
            des = !des;
        }

        if (des)
        {
            foreach (Transform child in parent)
            {
                if (!child.gameObject.GetComponent<MeshFilter>() && ! !child.gameObject.activeSelf)
                {
                    Destroy(child.gameObject);
                }
            }
        }
    }

    void SetNewParent(Transform obj)
    {
        if (obj.childCount != 0)
        {
            foreach (Transform _child in obj)
            {
                SetNewParent(_child);
            }
        }

        if (obj.parent != parent)
        {
            n++;
            obj.transform.SetParent(parent);
            obj.name = n.ToString();
        }
    }
}
