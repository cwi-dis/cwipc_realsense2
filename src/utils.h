//
//  utils.h
//
//  Created by Fons Kuijk on 12-12-18.
//

#ifndef cwipw_realsense_utils_h
#define cwipw_realsense_utils_h
#pragma once

typedef struct HsvColor
{
	unsigned char h;
	unsigned char s;
	unsigned char v;
} HsvColor;

cwipc_pcl_point* hsvToRgb(HsvColor hsv, cwipc_pcl_point* pnt)
{
	unsigned char region, p, q, t;
	unsigned int h, s, v, remainder;

	if (hsv.s == 0)
	{
		pnt->r = hsv.v;
		pnt->g = hsv.v;
		pnt->b = hsv.v;
		return pnt;
	}

	// converting to 16 bit to prevent overflow
	h = hsv.h;
	s = hsv.s;
	v = hsv.v;

	region = h / 43;
	remainder = (h - (region * 43)) * 6;

	p = (v * (255 - s)) >> 8;
	q = (v * (255 - ((s * remainder) >> 8))) >> 8;
	t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

	switch (region)
	{
	case 0:
		pnt->r = v;
		pnt->g = t;
		pnt->b = p;
		break;
	case 1:
		pnt->r = q;
		pnt->g = v;
		pnt->b = p;
		break;
	case 2:
		pnt->r = p;
		pnt->g = v;
		pnt->b = t;
		break;
	case 3:
		pnt->r = p;
		pnt->g = q;
		pnt->b = v;
		break;
	case 4:
		pnt->r = t;
		pnt->g = p;
		pnt->b = v;
		break;
	default:
		pnt->r = v;
		pnt->g = p;
		pnt->b = q;
		break;
	}

	return pnt;
}

HsvColor rgbToHsv(cwipc_pcl_point* pnt)
{
	HsvColor hsv;
	unsigned char rgbMin, rgbMax;

	rgbMin = pnt->r < pnt->g ? (pnt->r < pnt->b ? pnt->r : pnt->b) : (pnt->g < pnt->b ? pnt->g : pnt->b);
	rgbMax = pnt->r > pnt->g ? (pnt->r > pnt->b ? pnt->r : pnt->b) : (pnt->g > pnt->b ? pnt->g : pnt->b);

	hsv.v = rgbMax;
	if (hsv.v == 0)
	{
		hsv.h = 0;
		hsv.s = 0;
		return hsv;
	}

	hsv.s = 255 * ((long)(rgbMax - rgbMin)) / hsv.v;
	if (hsv.s == 0)
	{
		hsv.h = 0;
		return hsv;
	}

	if (rgbMax == pnt->r)
		hsv.h = 0 + 43 * (pnt->g - pnt->b) / (rgbMax - rgbMin);
	else if (rgbMax == pnt->g)
		hsv.h = 85 + 43 * (pnt->b - pnt->r) / (rgbMax - rgbMin);
	else
		hsv.h = 171 + 43 * (pnt->r - pnt->g) / (rgbMax - rgbMin);

	return hsv;
}

bool noChromaRemoval(cwipc_pcl_point* p)
{
	HsvColor hsv = rgbToHsv(p);

	if (hsv.h >= 60 && hsv.h <= 130) {
		if (hsv.s >= 0.15 && hsv.v >= 0.15) {
			// reducegreen
			if ((p->r * p->b) != 0 && (p->g * p->g) / (p->r * p->b) > 1.5) {
				p->r *= 1.4;
				p->b *= 1.4;
			}
			else {
				p->r *= 1.2;
				p->b *= 1.2;
			}
		}
		return !(hsv.s >= 0.4 && hsv.v >= 0.3);
	}
	return true;
}

#endif /* cwipw_realsense_utils_h */
