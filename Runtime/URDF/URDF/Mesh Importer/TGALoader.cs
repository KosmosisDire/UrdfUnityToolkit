// Extended from https://github.com/phwitti/EncodeToTGA/blob/master/EncodeToTGAExtension.cs
// ChatGPT added the decode funcitonality

using System.IO;
using UnityEngine;

namespace UrdfToolkit.Urdf.Importer
{
public static class TGALoader
{
    // == "TRUEVISION-XFile.\0" (ASCII)
    static readonly byte[] c_arV2Signature = { 0x54, 0x52, 0x55, 0x45, 0x56, 0x49, 0x53, 0x49, 0x4F, 0x4E, 0x2D, 0x58, 0x46, 0x49, 0x4C, 0x45, 0x2E, 0x00 };

    public enum Compression
    {
        None, RLE
    }

    public static byte[] EncodeToTGA(this Texture2D _texture2D, Compression _compression = Compression.RLE)
    {
        const int iTgaHeaderSize = 18;
        const int iBytesPerPixelRGB24 = 3; // 1 byte per channel (rgb)
        const int iBytesPerPixelARGB32 = 4; // ~ (rgba)

        int iBytesPerPixel = _texture2D.format == TextureFormat.ARGB32 || _texture2D.format == TextureFormat.RGBA32
            ? iBytesPerPixelARGB32
            : iBytesPerPixelRGB24;

        using (MemoryStream memoryStream = new MemoryStream(iTgaHeaderSize + _texture2D.width * _texture2D.height * iBytesPerPixel))
        {
            using (BinaryWriter binaryWriter = new BinaryWriter(memoryStream))
            {
                // Write TGA Header
                
                binaryWriter.Write((byte)0);                    // IDLength (not in use)
                binaryWriter.Write((byte)0);                    // ColorMapType (not in use)
                binaryWriter.Write((byte)(_compression == Compression.None ? 2 : 10)); // DataTypeCode (10 == Runlength encoded RGB images)
                binaryWriter.Write((short)0);                   // ColorMapOrigin (not in use)
                binaryWriter.Write((short)0);                   // ColorMapLength (not in use)
                binaryWriter.Write((byte)0);                    // ColorMapDepth (not in use)
                binaryWriter.Write((short)0);                   // Origin X
                binaryWriter.Write((short)0);                   // Origin Y
                binaryWriter.Write((short)_texture2D.width);    // Width
                binaryWriter.Write((short)_texture2D.height);   // Height
                binaryWriter.Write((byte)(iBytesPerPixel * 8)); // Bits Per Pixel
                binaryWriter.Write((byte)0);                    // ImageDescriptor (not in use)

                Color32[] arPixels = _texture2D.GetPixels32();

                if (_compression == Compression.None)
                {
                    // Write all Pixels one after the other
                    for (int iPixel = 0; iPixel < arPixels.Length; iPixel++)
                    {
                        Color32 c32Pixel = arPixels[iPixel];
                        binaryWriter.Write(c32Pixel.b);
                        binaryWriter.Write(c32Pixel.g);
                        binaryWriter.Write(c32Pixel.r);

                        if (iBytesPerPixel == iBytesPerPixelARGB32)
                            binaryWriter.Write(c32Pixel.a);
                    }
                }
                else
                {
                    // Write RLE Encoded Pixels

                    const int iMaxPacketLength = 128;
                    int iPacketStart = 0;
                    int iPacketEnd = 0;

                    while (iPacketStart < arPixels.Length)
                    {
                        Color32 c32PreviousPixel = arPixels[iPacketStart];

                        // Get current Packet Type
                        RLEPacketType packetType = TGALoader.PacketType(arPixels, iPacketStart);

                        // Find Packet End
                        int iReadEnd = Mathf.Min(iPacketStart + iMaxPacketLength, arPixels.Length);
                        for (iPacketEnd = iPacketStart + 1; iPacketEnd < iReadEnd; ++iPacketEnd)
                        {
                            bool bPreviousEqualsCurrent = TGALoader.Equals(arPixels[iPacketEnd - 1], arPixels[iPacketEnd]);

                            // Packet End if change in Packet Type or if max Packet-Size reached
                            if (packetType == RLEPacketType.RAW && bPreviousEqualsCurrent ||
                                packetType == RLEPacketType.RLE && !bPreviousEqualsCurrent)
                            {
                                break;
                            }
                        }

                        // Write Packet

                        int iPacketLength = iPacketEnd - iPacketStart;

                        switch (packetType)
                        {
                            case RLEPacketType.RLE:

                                // Add RLE-Bit to PacketLength
                                binaryWriter.Write((byte)((iPacketLength - 1) | (1 << 7)));

                                binaryWriter.Write(c32PreviousPixel.b);
                                binaryWriter.Write(c32PreviousPixel.g);
                                binaryWriter.Write(c32PreviousPixel.r);

                                if (iBytesPerPixel == iBytesPerPixelARGB32)
                                    binaryWriter.Write(c32PreviousPixel.a);

                                break;
                            case RLEPacketType.RAW:

                                binaryWriter.Write((byte)(iPacketLength - 1));

                                for (int iPacketPosition = iPacketStart; iPacketPosition < iPacketEnd; ++iPacketPosition)
                                {
                                    Color32 c32Pixel = arPixels[iPacketPosition];
                                    binaryWriter.Write(c32Pixel.b);
                                    binaryWriter.Write(c32Pixel.g);
                                    binaryWriter.Write(c32Pixel.r);

                                    if (iBytesPerPixel == iBytesPerPixelARGB32)
                                        binaryWriter.Write(c32Pixel.a);
                                }

                                break;
                        }

                        iPacketStart = iPacketEnd;
                    }
                }

                binaryWriter.Write(0);          // Offset of meta-information (not in use)
                binaryWriter.Write(0);          // Offset of Developer-Area (not in use)
                binaryWriter.Write(c_arV2Signature); // ImageDescriptor (not in use)
            }

            return memoryStream.ToArray();
        }
    }

    public static Texture2D DecodeTGA(byte[] tgaData)
    {
        // const int iTgaHeaderSize = 18;
        const int iBytesPerPixelRGB24 = 3; // 1 byte per channel (rgb)
        const int iBytesPerPixelARGB32 = 4; // ~ (rgba)

        using (MemoryStream memoryStream = new MemoryStream(tgaData))
        {
            using (BinaryReader binaryReader = new BinaryReader(memoryStream))
            {
                // Read TGA Header
                binaryReader.ReadByte(); // IDLength (not in use)
                binaryReader.ReadByte(); // ColorMapType (not in use)
                byte dataTypeCode = binaryReader.ReadByte(); // DataTypeCode (2 == Uncompressed, 10 == RLE compressed)
                binaryReader.ReadInt16(); // ColorMapOrigin (not in use)
                binaryReader.ReadInt16(); // ColorMapLength (not in use)
                binaryReader.ReadByte(); // ColorMapDepth (not in use)
                binaryReader.ReadInt16(); // Origin X
                binaryReader.ReadInt16(); // Origin Y
                short width = binaryReader.ReadInt16(); // Width
                short height = binaryReader.ReadInt16(); // Height
                byte bitsPerPixel = binaryReader.ReadByte(); // Bits Per Pixel
                binaryReader.ReadByte(); // ImageDescriptor (not in use)

                int iBytesPerPixel = bitsPerPixel == 32 ? iBytesPerPixelARGB32 : iBytesPerPixelRGB24;

                Texture2D texture2D = new Texture2D(width, height, bitsPerPixel == 32 ? TextureFormat.RGBA32 : TextureFormat.RGB24, false);
                Color32[] arPixels = new Color32[width * height];

                if (dataTypeCode == 2)
                {
                    // Uncompressed
                    for (int iPixel = 0; iPixel < arPixels.Length; iPixel++)
                    {
                        byte b = binaryReader.ReadByte();
                        byte g = binaryReader.ReadByte();
                        byte r = binaryReader.ReadByte();
                        byte a = bitsPerPixel == 32 ? binaryReader.ReadByte() : (byte)255;

                        arPixels[iPixel] = new Color32(r, g, b, a);
                    }
                }
                else if (dataTypeCode == 10)
                {
                    // RLE compressed
                    int iPixel = 0;
                    while (iPixel < arPixels.Length)
                    {
                        byte packetHeader = binaryReader.ReadByte();
                        int packetSize = (packetHeader & 0x7F) + 1;

                        if ((packetHeader & 0x80) != 0)
                        {
                            // RLE packet
                            byte b = binaryReader.ReadByte();
                            byte g = binaryReader.ReadByte();
                            byte r = binaryReader.ReadByte();
                            byte a = bitsPerPixel == 32 ? binaryReader.ReadByte() : (byte)255;

                            Color32 color = new Color32(r, g, b, a);
                            for (int i = 0; i < packetSize; i++)
                            {
                                arPixels[iPixel++] = color;
                            }
                        }
                        else
                        {
                            // Raw packet
                            for (int i = 0; i < packetSize; i++)
                            {
                                byte b = binaryReader.ReadByte();
                                byte g = binaryReader.ReadByte();
                                byte r = binaryReader.ReadByte();
                                byte a = bitsPerPixel == 32 ? binaryReader.ReadByte() : (byte)255;

                                arPixels[iPixel++] = new Color32(r, g, b, a);
                            }
                        }
                    }
                }

                texture2D.SetPixels32(arPixels);
                texture2D.Apply();
                return texture2D;
            }
        }
    }

    private enum RLEPacketType { RLE, RAW }

    private static bool Equals(Color32 _first, Color32 _second)
    {
        return _first.r == _second.r && _first.g == _second.g && _first.b == _second.b && _first.a == _second.a;
    }

    private static RLEPacketType PacketType(Color32[] _arData, int _iPacketPosition)
    {
        if ((_iPacketPosition != _arData.Length - 1) && TGALoader.Equals(_arData[_iPacketPosition], _arData[_iPacketPosition + 1]))
        {
            return RLEPacketType.RLE;
        }
        else
        {
            return RLEPacketType.RAW;
        }
    }
}
}
