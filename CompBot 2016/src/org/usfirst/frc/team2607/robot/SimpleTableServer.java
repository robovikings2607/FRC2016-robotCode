package org.usfirst.frc.team2607.robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.net.MulticastSocket;
import java.net.NetworkInterface;
import java.text.DecimalFormat;
import java.util.HashMap;
import java.util.Iterator;


public class SimpleTableServer extends Thread {

	
	HashMap<String,String> pairs = new HashMap<String,String>();
	int port = 5810;
	MulticastSocket socket = new MulticastSocket(port); 
	InetAddress group;
	DecimalFormat floatFmt = new DecimalFormat("##.00");
	
	public SimpleTableServer() throws Exception
	{
		group = InetAddress.getByName("226.226.226.226");
		start();
	}
	
	public void put(String key, String value)
	{
		try {
			byte[] buf = new String(key+"~"+value).trim().getBytes();
			DatagramPacket packet = new DatagramPacket(buf, buf.length, group, port);
			socket.send(packet);
		} catch (IOException e) {
			System.out.println("error sending data");
			e.printStackTrace();
		}
	}
	
	public void put(String key, int value)
	{
		put(key, String.valueOf(value));
	}

	public void put(String key, double value) {
		put(key, floatFmt.format(value));
	}
	
	public String get(String key)
	
	{
		return pairs.get(key);
	}
	
	public void run()
	{
		try
		{
			socket.joinGroup(group);
			while ( true )
			{
				byte[] buf = new byte[256];
				DatagramPacket packet = new DatagramPacket(buf, buf.length);
			    socket.receive(packet);
			    
			    String data = new String(packet.getData());
			    String[] keyvalue = data.split("~");
			    if ( keyvalue.length == 2 )
			    	pairs.put(keyvalue[0], keyvalue[1]);
			}
		} catch (Exception e) {
			e.printStackTrace();
			System.out.println("Server error!");
		}
	}
}