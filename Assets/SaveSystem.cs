using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SaveSystem
{
    public static readonly string SAVE_FOLDER = Application.dataPath + "/Save/";

    public static void Init()
    {
        //Debug.Log(Application.dataPath);
        //Debug.Log(Application.persistentDataPath);

        if (!Directory.Exists(SAVE_FOLDER))
        {
            Directory.CreateDirectory(SAVE_FOLDER);
        }    
    }

    public static void Save(string saveString, string name)
    {
        int saveNumber = 1;
        while (File.Exists(SAVE_FOLDER + name + saveNumber + ".json"))
        {
            saveNumber++;
        }

        Debug.Log("Saving...");
        File.WriteAllText(SAVE_FOLDER + name + saveNumber + ".json", saveString);
    }
}
