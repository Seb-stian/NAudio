using System;

namespace NAudio.Codecs;

/// <summary>
/// SpanDSP - a series of DSP components for telephony
/// 
/// g722_decode.c - The ITU G.722 codec, decode part.
/// 
/// Written by Steve Underwood &lt;steveu@coppice.org&gt;
/// 
/// Copyright (C) 2005 Steve Underwood
/// Ported to C# by Mark Heath 2011
/// 
/// Despite my general liking of the GPL, I place my own contributions 
/// to this code in the public domain for the benefit of all mankind -
/// even the slimy ones who might try to proprietize my work and use it
/// to my detriment.
///  
/// Based in part on a single channel G.722 codec which is:
/// Copyright (c) CMU 1993
/// Computer Science, Speech Group
/// Chengxiang Lu and Alex Hauptmann
/// 
/// Edited in 2026 by Seb-stian for general cleanup and optimization.
/// </summary>
public class G722Codec
{
    private static ReadOnlySpan<int> wl => [-60, -30, 58, 172, 334, 538, 1198, 3042];
    private static ReadOnlySpan<int> rl42 => [0, 7, 6, 5, 4, 3, 2, 1, 7, 6, 5, 4, 3, 2, 1, 0];
    private static ReadOnlySpan<int> ilb => [2048, 2093, 2139, 2186, 2233, 2282, 2332, 2383, 2435, 2489, 2543, 2599, 2656, 2714, 2774, 2834, 2896, 2960, 3025, 3091, 3158, 3228, 3298, 3371, 3444, 3520, 3597, 3676, 3756, 3838, 3922, 4008];
    private static ReadOnlySpan<int> wh => [0, -214, 798];
    private static ReadOnlySpan<int> rh2 => [2, 1, 2, 1];
    private static ReadOnlySpan<int> qm2 => [-7408, -1616, 7408, 1616];
    private static ReadOnlySpan<int> qm4 => [0, -20456, -12896, -8968, -6288, -4240, -2584, -1200, 20456, 12896, 8968, 6288, 4240, 2584, 1200, 0];
    private static ReadOnlySpan<int> qm5 => [-280, -280, -23352, -17560, -14120, -11664, -9752, -8184, -6864, -5712, -4696, -3784, -2960, -2208, -1520, -880, 23352, 17560, 14120, 11664, 9752, 8184, 6864, 5712, 4696, 3784, 2960, 2208, 1520, 880, 280, -280];
    private static ReadOnlySpan<int> qm6 => [-136, -136, -136, -136, -24808, -21904, -19008, -16704, -14984, -13512, -12280, -11192, -10232, -9360, -8576, -7856, -7192, -6576, -6000, -5456, -4944, -4464, -4008, -3576, -3168, -2776, -2400, -2032, -1688, -1360, -1040, -728, 24808, 21904, 19008, 16704, 14984, 13512, 12280, 11192, 10232, 9360, 8576, 7856, 7192, 6576, 6000, 5456, 4944, 4464, 4008, 3576, 3168, 2776, 2400, 2032, 1688, 1360, 1040, 728, 432, 136, -432, -136];
    private static ReadOnlySpan<int> qmf_coeffs => [3, -11, 12, 32, -210, 951, 3876, -805, 362, -156, 53, -11];
    private static ReadOnlySpan<int> q6 => [0, 35, 72, 110, 150, 190, 233, 276, 323, 370, 422, 473, 530, 587, 650, 714, 786, 858, 940, 1023, 1121, 1219, 1339, 1458, 1612, 1765, 1980, 2195, 2557, 2919, 0, 0];
    private static ReadOnlySpan<int> iln => [0, 63, 62, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 0];
    private static ReadOnlySpan<int> ilp => [0, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 0];
    private static ReadOnlySpan<int> ihn => [0, 1, 0];
    private static ReadOnlySpan<int> ihp => [0, 3, 2];

    private static void Block4(G722CodecState s, int bandIndex, int d)
    {
        ref readonly Band band = ref s.Band[bandIndex];

        // Block 4, RECONS
        band.d[0] = d;
        band.r[0] = Saturate(band.s + d);

        // Block 4, PARREC
        band.p[0] = Saturate(band.sz + d);

        // Block 4, UPPOL2
        for (int i = 0; i < 3; i++)
            band.sg[i] = band.p[i] >> 15;
        int wd1 = Saturate(band.a[1] << 2);

        int wd2 = (band.sg[0] == band.sg[1]) ? -wd1 : wd1;
        if (wd2 > 32767)
            wd2 = 32767;
        int wd3 = (band.sg[0] == band.sg[2]) ? 128 : -128;
        wd3 += (wd2 >> 7);
        wd3 += (band.a[2] * 32512) >> 15;
        wd3 = Math.Clamp(wd3, -12288, 12288);
        band.ap[2] = wd3;

        // Block 4, UPPOL1
        band.sg[0] = band.p[0] >> 15;
        band.sg[1] = band.p[1] >> 15;
        wd1 = (band.sg[0] == band.sg[1]) ? 192 : -192;
        wd2 = (band.a[1] * 32640) >> 15;

        band.ap[1] = Saturate(wd1 + wd2);
        wd3 = Saturate(15360 - band.ap[2]);
        band.ap[1] = Math.Clamp(band.ap[1], -wd3, wd3);

        // Block 4, UPZERO
        wd1 = (d == 0) ? 0 : 128;
        band.sg[0] = d >> 15;
        for (int i = 1; i < 7; i++)
        {
            band.sg[i] = band.d[i] >> 15;
            wd2 = (band.sg[i] == band.sg[0]) ? wd1 : -wd1;
            wd3 = (band.b[i] * 32640) >> 15;
            band.bp[i] = Saturate(wd2 + wd3);
        }

        // Block 4, DELAYA
        for (int i = 6; i > 0; i--)
        {
            band.d[i] = band.d[i - 1];
            band.b[i] = band.bp[i];
        }

        for (int i = 2; i > 0; i--)
        {
            band.r[i] = band.r[i - 1];
            band.p[i] = band.p[i - 1];
            band.a[i] = band.ap[i];
        }

        // Block 4, FILTEP
        wd1 = Saturate(band.r[1] + band.r[1]);
        wd1 = (band.a[1] * wd1) >> 15;
        wd2 = Saturate(band.r[2] + band.r[2]);
        wd2 = (band.a[2] * wd2) >> 15;
        band.sp = Saturate(wd1 + wd2);

        // Block 4, FILTEZ
        int sz = 0;
        for (int i = 6; i > 0; i--)
        {
            wd1 = Saturate(band.d[i] + band.d[i]);
            sz += (band.b[i] * wd1) >> 15;
        }
        band.sz = Saturate(sz);

        // Block 4, PREDIC
        band.s = Saturate(band.sp + band.sz);
    }

    /// <summary>
    /// Decodes a buffer of G722
    /// </summary>
    /// <param name="state">Codec state</param>
    /// <param name="outputBuffer">Output buffer (to contain decompressed PCM samples)</param>
    /// <param name="inputG722Data"></param>
    /// <param name="inputLength">Number of bytes in input G722 data to decode</param>
    /// <returns>Number of samples written into output buffer</returns>
    public int Decode(G722CodecState state, short[] outputBuffer, byte[] inputG722Data, int inputLength)
    {
        int bytesWritten = 0;
        for (int i = 0; i < inputLength;)
        {
            int code;
            if (state.Packed)
            {
                // Unpack the code bits
                if (state.InBits < state.BitsPerSample)
                {
                    state.InBuffer |= (uint)(inputG722Data[i++] << state.InBits);
                    state.InBits += 8;
                }
                code = (int)state.InBuffer & ((1 << state.BitsPerSample) - 1);
                state.InBuffer >>= state.BitsPerSample;
                state.InBits -= state.BitsPerSample;
            }
            else
            {
                code = inputG722Data[i++];
            }

            int wd1, wd2, wd3;
            int ihigh, rhigh = 0, rlow;
            switch (state.BitsPerSample)
            {
                default:
                case 8:
                    wd1 = code & 0x3F;
                    ihigh = (code >> 6) & 0x03;
                    wd2 = qm6[wd1];
                    wd1 >>= 2;
                    break;
                case 7:
                    wd1 = code & 0x1F;
                    ihigh = (code >> 5) & 0x03;
                    wd2 = qm5[wd1];
                    wd1 >>= 1;
                    break;
                case 6:
                    wd1 = code & 0x0F;
                    ihigh = (code >> 4) & 0x03;
                    wd2 = qm4[wd1];
                    break;
            }

            // Block 5L, LOW BAND INVQBL
            wd2 = (state.Band[0].det * wd2) >> 15;

            // Block 5L, RECONS
            rlow = state.Band[0].s + wd2;

            // Block 6L, LIMIT
            rlow = Math.Clamp(rlow, -16384, 16383);

            // Block 2L, INVQAL
            wd2 = qm4[wd1];
            int dlowt = (state.Band[0].det * wd2) >> 15;

            // Block 3L, LOGSCL
            wd2 = rl42[wd1];
            wd1 = (state.Band[0].nb * 127) >> 7;
            wd1 += wl[wd2];
            wd1 = Math.Clamp(wd1, 0, 18432);
            state.Band[0].nb = wd1;

            // Block 3L, SCALEL
            wd1 = (state.Band[0].nb >> 6) & 31;
            wd2 = 8 - (state.Band[0].nb >> 11);
            wd3 = (wd2 < 0) ? (ilb[wd1] << -wd2) : (ilb[wd1] >> wd2);
            state.Band[0].det = wd3 << 2;

            Block4(state, 0, dlowt);

            if (!state.EncodeFrom8000Hz)
            {
                // Block 2H, INVQAH
                wd2 = qm2[ihigh];
                int dhigh = (state.Band[1].det * wd2) >> 15;

                // Block 5H, RECONS
                rhigh = dhigh + state.Band[1].s;

                // Block 6H, LIMIT
                rhigh = Math.Clamp(rhigh, -16384, 16383);

                // Block 2H, INVQAH
                wd2 = rh2[ihigh];
                wd1 = (state.Band[1].nb * 127) >> 7;
                wd1 += wh[wd2];
                wd1 = Math.Clamp(wd1, 0, 22528);
                state.Band[1].nb = wd1;

                // Block 3H, SCALEH
                wd1 = (state.Band[1].nb >> 6) & 31;
                wd2 = 10 - (state.Band[1].nb >> 11);
                wd3 = (wd2 < 0) ? (ilb[wd1] << -wd2) : (ilb[wd1] >> wd2);
                state.Band[1].det = wd3 << 2;

                Block4(state, 1, dhigh);
            }

            if (state.ItuTestMode)
            {
                outputBuffer[bytesWritten++] = (short)(rlow << 1);
                outputBuffer[bytesWritten++] = (short)(rhigh << 1);
            }
            else
            {
                if (state.EncodeFrom8000Hz)
                {
                    outputBuffer[bytesWritten++] = (short)(rlow << 1);
                }
                else
                {
                    // Apply the receive QMF
                    for (int j = 0; j < 22; j++)
                    {
                        state.QmfSignalHistory[j] = state.QmfSignalHistory[j + 2];
                    }
                    state.QmfSignalHistory[22] = rlow + rhigh;
                    state.QmfSignalHistory[23] = rlow - rhigh;

                    // Even and odd tap accumulators
                    int sumOdd = 0;
                    int sumEven = 0;

                    for (int j = 0; j < 12; j++)
                    {
                        sumEven += state.QmfSignalHistory[2 * j] * qmf_coeffs[j];
                        sumOdd += state.QmfSignalHistory[2 * j + 1] * qmf_coeffs[11 - j];
                    }
                    outputBuffer[bytesWritten++] = (short)(sumOdd >> 11);
                    outputBuffer[bytesWritten++] = (short)(sumEven >> 11);
                }
            }
        }
        return bytesWritten;
    }

    /// <summary>
    /// Encodes a buffer of G722
    /// </summary>
    /// <param name="state">Codec state</param>
    /// <param name="outputBuffer">Output buffer (to contain encoded G722)</param>
    /// <param name="inputBuffer">PCM 16 bit samples to encode</param>
    /// <param name="inputBufferCount">Number of samples in the input buffer to encode</param>
    /// <returns>Number of encoded bytes written into output buffer</returns>
    public int Encode(G722CodecState state, byte[] outputBuffer, short[] inputBuffer, int inputBufferCount)
    {
        int g722Bytes = 0;

        Band firstBand = state.Band[0];
        Band secondBand = state.Band[1];

        for (int i = 0; i < inputBufferCount;)
        {
            // Low and high band PCM from the QMF
            (int xLow, int xHigh) = GetPcmBandFromQmf(state, inputBuffer, ref i);

            // Block 1L, SUBTRA
            int el = Saturate(xLow - firstBand.s);

            // Block 1L, QUANTL
            int wd = (el >= 0) ? el : -(el + 1);
            int wd1;
            int index;
            for (index = 1; index < 30; index++)
            {
                wd1 = (q6[index] * firstBand.det) >> 12;
                if (wd < wd1)
                    break;
            }
            int iLow = (el < 0) ? iln[index] : ilp[index];

            // Block 2L, INVQAL
            int ril = iLow >> 2;
            int wd2 = qm4[ril];
            int dlow = (firstBand.det * wd2) >> 15;

            // Block 3L, LOGSCL
            int il4 = rl42[ril];
            wd = (firstBand.nb * 127) >> 7;
            firstBand.nb = Math.Clamp(wd + wl[il4], 0, 18432);

            // Block 3L, SCALEL
            wd1 = (firstBand.nb >> 6) & 31;
            wd2 = 8 - (firstBand.nb >> 11);
            int wd3 = (wd2 < 0) ? (ilb[wd1] << -wd2) : (ilb[wd1] >> wd2);
            firstBand.det = wd3 << 2;

            Block4(state, 0, dlow);

            int code = GetEncodingCode(state, iLow, xHigh, secondBand);
            if (state.Packed)
            {
                // Pack the code bits
                state.OutBuffer |= (uint)(code << state.OutBits);
                state.OutBits += state.BitsPerSample;
                if (state.OutBits >= 8)
                {
                    outputBuffer[g722Bytes++] = (byte)(state.OutBuffer & 0xFF);
                    state.OutBits -= 8;
                    state.OutBuffer >>= 8;
                }
            }
            else
            {
                outputBuffer[g722Bytes++] = (byte)code;
            }
        }
        return g722Bytes;
    }

    private static (int xLow, int xHigh) GetPcmBandFromQmf(G722CodecState state, ReadOnlySpan<short> inputBuffer, ref int index)
    {
        if (state.ItuTestMode)
        {
            int pcm = inputBuffer[index++] >> 1;
            return (pcm, pcm);
        }

        if (state.EncodeFrom8000Hz)
        {
            int pcm = inputBuffer[index++] >> 1;
            return (pcm, 0);
        }

        // Apply the transmit QMF
        // Shuffle the buffer down
        for (int i = 0; i < 22; i++)
        {
            state.QmfSignalHistory[i] = state.QmfSignalHistory[i + 2];
        }
        state.QmfSignalHistory[22] = inputBuffer[index++];
        state.QmfSignalHistory[23] = inputBuffer[index++];

        // Even and odd tap accumulators
        int sumEven = 0;
        int sumOdd = 0;

        // Discard every other QMF output
        for (int i = 0; i < 12; i++)
        {
            sumOdd += state.QmfSignalHistory[2 * i] * qmf_coeffs[i];
            sumEven += state.QmfSignalHistory[2 * i + 1] * qmf_coeffs[11 - i];
        }

        int xLow = (sumEven + sumOdd) >> 14;
        int xHigh = (sumEven - sumOdd) >> 14;

        return (xLow, xHigh);
    }

    private static int GetEncodingCode(G722CodecState state, int iLow, int xHigh, Band band)
    {
        if (state.EncodeFrom8000Hz)
        {
            // Just leave the high bits as zero
            return (0xC0 | iLow) >> (8 - state.BitsPerSample);
        }

        // Block 1H, SUBTRA
        int eh = Saturate(xHigh - band.s);

        // Block 1H, QUANTH
        int wd = (eh >= 0) ? eh : -(eh + 1);
        int wd1 = (564 * band.det) >> 12;
        int mih = (wd >= wd1) ? 2 : 1;
        int iHigh = (eh < 0) ? ihn[mih] : ihp[mih];

        // Block 2H, INVQAH
        int wd2 = qm2[iHigh];
        int dHigh = (band.det * wd2) >> 15;

        // Block 3H, LOGSCH
        int ih2 = rh2[iHigh];
        wd = (band.nb * 127) >> 7;
        band.nb = Math.Clamp(wd + wh[ih2], 0, 22528);

        // Block 3H, SCALEH
        wd1 = (band.nb >> 6) & 31;
        wd2 = 10 - (band.nb >> 11);
        int wd3 = (wd2 < 0) ? (ilb[wd1] << -wd2) : (ilb[wd1] >> wd2);
        band.det = wd3 << 2;

        Block4(state, 1, dHigh);
        return ((iHigh << 6) | iLow) >> (8 - state.BitsPerSample);
    }

    /// <summary>
    /// Hard limits to 16 bit samples
    /// </summary>
    private static short Saturate(int amp)
    {
        return short.CreateSaturating(amp);
    }
}

/// <summary>
/// Stores state to be used between calls to Encode or Decode
/// </summary>
public class G722CodecState
{
    /// <summary>
    /// ITU Test Mode
    /// TRUE if the operating in the special ITU test mode, with the band split filters disabled.
    /// </summary>
    public bool ItuTestMode { get; set; }

    /// <summary>
    /// TRUE if the G.722 data is packed
    /// </summary>
    public bool Packed { get; private set; }

    /// <summary>
    /// 8kHz Sampling
    /// TRUE if encode from 8k samples/second
    /// </summary>
    public bool EncodeFrom8000Hz { get; private set; }

    /// <summary>
    /// Bits Per Sample
    /// 6 for 48000kbps, 7 for 56000kbps, or 8 for 64000kbps.
    /// </summary>
    public int BitsPerSample { get; private set; }

    /// <summary>
    /// Signal history for the QMF (x)
    /// </summary>
    public int[] QmfSignalHistory { get; private set; }

    /// <summary>
    /// Band
    /// </summary>
    public Band[] Band { get; private set; }

    /// <summary>
    /// In bit buffer
    /// </summary>
    public uint InBuffer { get; internal set; }

    /// <summary>
    /// Number of bits in InBuffer
    /// </summary>
    public int InBits { get; internal set; }

    /// <summary>
    /// Out bit buffer
    /// </summary>
    public uint OutBuffer { get; internal set; }

    /// <summary>
    /// Number of bits in OutBuffer
    /// </summary>
    public int OutBits { get; internal set; }

    /// <summary>
    /// Creates a new instance of G722 Codec State for a 
    /// new encode or decode session
    /// </summary>
    /// <param name="rate">Bitrate; one of 48000, 56000, or (typically) 64000</param>
    /// <param name="options">Special options</param>
    public G722CodecState(int rate, G722Flags options)
    {
        Band = [new(), new()];
        QmfSignalHistory = new int[24];
        ItuTestMode = false;

        BitsPerSample = rate switch
        {
            48000 => 6,
            56000 => 7,
            64000 => 8,
            _ => throw new ArgumentException("Invalid rate, should be 48000, 56000 or 64000")
        };

        EncodeFrom8000Hz = (options & G722Flags.SampleRate8000) != 0;
        Packed = (options & G722Flags.Packed) != 0 && BitsPerSample != 8;

        Band[0].det = 32;
        Band[1].det = 8;
    }
}

/// <summary>
/// Band data for G722 Codec
/// </summary>
public class Band
{
    /// <summary>s - Signal estimate.</summary>
    public int s;
    /// <summary>sp - Pole-section predictor.</summary>
    public int sp;
    /// <summary>sz - Zero-section predictor.</summary>
    public int sz;
    /// <summary>r - Reconstructed signals.</summary>
    public int[] r = new int[3];
    /// <summary>a - Second-order pole predictor coefficients.</summary>
    public int[] a = new int[3];
    /// <summary>ap - Pending pole coefficients.</summary>
    public int[] ap = new int[3];
    /// <summary>p - Partially reconstructed signals.</summary>
    public int[] p = new int[3];
    /// <summary>d - Difference signals.</summary>
    public int[] d = new int[7];
    /// <summary>b - Sixth-order zero predictor coefficients.</summary>
    public int[] b = new int[7];
    /// <summary>bp - Pending zero predictor coefficients.</summary>
    public int[] bp = new int[7];
    /// <summary>sg - Sign bits of difference signals.</summary>
    public int[] sg = new int[7];
    /// <summary>nb - Logarithmic quantizer scale factor.</summary>
    public int nb;
    /// <summary>det - Linear quantizer scale factor.</summary>
    public int det;
}

/// <summary>
/// G722 Flags
/// </summary>
[Flags]
public enum G722Flags
{
    /// <summary>
    /// None
    /// </summary>
    None = 0,
    /// <summary>
    /// Using a G722 sample rate of 8000
    /// </summary>
    SampleRate8000 = 0x0001,
    /// <summary>
    /// Packed
    /// </summary>
    Packed = 0x0002
}
