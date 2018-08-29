using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ZEDCam : MonoBehaviour {

	private WebCamTexture webcamTexture;
	private Renderer renderer;

	[SerializeField] private Material webCamTex;
	

	// Use this for initialization
	void Start () {
		webCamTex= Resources.Load("webCamTex", typeof(Material)) as Material;

		WebCamDevice[] devices= WebCamTexture.devices;

		string camName= devices[1].name;

		WebCamTexture camFeed= new WebCamTexture(camName);

		webCamTex.mainTexture= camFeed;

		camFeed.Play();

		gameObject.GetComponent<Renderer>().material=webCamTex;

	}
	
	// Update is called once per frame
	void Update () {
		
	}
}
