using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Display : MonoBehaviour
{
    public Text nameIndex;

    // Start is called before the first frame update
    void Start()
    {
        nameIndex = gameObject.GetComponent<Text>();
        Invoke("UpdateName", 2.5f);
    }

    // Update is called once per frame
    void Update()
    {

    }

    void UpdateName()
    {
        nameIndex.text = transform.parent.parent.name.Split('_')[1];
    }
}
