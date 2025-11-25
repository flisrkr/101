//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType
{
    DIFFUSE,
    MICROFACET,
};

class Material
{
private:
    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0)
        {
            cosi = -cosi;
        }
        else
        {
            std::swap(etai, etat);
            n = -N;
        }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0)
        {
            std::swap(etai, etat);
        }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1)
        {
            kr = 1;
        }
        else
        {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    Vector3f toWorld(const Vector3f &a, const Vector3f &N)
    {
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y))
        {
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x * invLen);
        }
        else
        {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y * invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }
    // new functions to implement microfacet and PBR pipeline
    inline Vector3f fresnelSchlick(const float h_dot_wi, const Vector3f &F_0);
    inline float geometrySchlickSmith(const float N_dot_w);
    inline float distributionGGX(const float N_dot_h);

public:
    MaterialType m_type;
    Vector3f m_emission;
    float ior;
    // Vector3f Kd;

    // new attributes to implement microfacet and PBR pipeline
    float roughness;
    float metallic;
    Vector3f albedo;
    Vector3f F_0;
    float alpha;
    float alpha_square;
    float k;

    inline Material(MaterialType t = DIFFUSE, Vector3f e = Vector3f(0, 0, 0));
    inline Material(MaterialType t, const Vector3f &e, float r, float m, const Vector3f &a);
    inline MaterialType getType();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
};

Material::Material(MaterialType t, Vector3f e) : m_type(t), m_emission(e) {}

Material::Material(MaterialType t, const Vector3f &e, float r, float m, const Vector3f &a)
    : m_type(t), m_emission(e), roughness(r), metallic(m), albedo(a)
{
    F_0 = lerp(Vector3f(0.04f), albedo, metallic);
    alpha = std::max(EPSILON, roughness * roughness);
    alpha_square = alpha * alpha;
    k = (roughness + 1.f) * (roughness + 1.f) / 8.f;
}

MaterialType Material::getType() { return m_type; }
/// Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() { return m_emission; }
bool Material::hasEmission()
{
    if (m_emission.norm() > EPSILON)
        return true;
    else
        return false;
}

Vector3f Material::getColorAt(double u, double v)
{
    return Vector3f();
}

Vector3f Material::sample(const Vector3f &wi, const Vector3f &N)
{
    switch (m_type)
    {
    case DIFFUSE:
    {
        // uniform sample on the hemisphere
        float x_1 = get_random_float(), x_2 = get_random_float();
        float r = sqrt(x_1);
        float theta = 2.f * M_PI * x_2;
        Vector3f localRay(r * cos(theta), r * sin(theta), sqrt(std::max(0.f, 1 - x_1)));
        return toWorld(localRay, N);

        break;
    }
    case MICROFACET:
    {
        float diffuse_prob = clamp(0.05f, 0.95f, 1.f - metallic);
        if (get_random_float() < diffuse_prob)
        {
            // sample diffuse component
            float x_1 = get_random_float(), x_2 = get_random_float();
            float r = sqrt(x_1);
            float theta = 2.f * M_PI * x_2;
            Vector3f localRay(r * cos(theta), r * sin(theta), sqrt(std::max(0.f, 1 - x_1)));
            return toWorld(localRay, N);
        }
        float u_1 = get_random_float();
        float u_2 = get_random_float();

        float cos_theta = sqrt((1 - u_1) / (1 + (alpha - 1) * u_1));
        float sin_theta = sqrt(std::max(0.f, 1.f - cos_theta * cos_theta));
        float phi = 2.f * M_PI * u_2;

        Vector3f h_local(sin_theta * cos(phi), sin_theta * sin(phi), cos_theta);
        Vector3f h = toWorld(h_local, N);

        return reflect(-wi, h);

        break;
    }
    }
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N)
{
    switch (m_type)
    {
    case DIFFUSE:
    {
        // uniform sample probability 1 / (2 * PI)
        if (dotProduct(wo, N) > 0.0f)
            return 0.5f / M_PI;
        else
            return 0.0f;
        break;
    }
    case MICROFACET:
    {
        float diffuse_prob = clamp(0.05f, 0.95f, 1.f - metallic);
        float cos_theta = dotProduct(N, wo);
        float pdf_diffuse = std::max(0.f, cos_theta) / M_PI;
        Vector3f h = (wi + wo).normalized();
        float N_dot_h = std::max(0.f, dotProduct(N, h));
        float h_dot_wo = std::max(0.f, dotProduct(h, wo));
        float D_term = distributionGGX(N_dot_h);
        float pdf_specular = std::max(EPSILON, (D_term * N_dot_h) / (4.f * h_dot_wo));
        return diffuse_prob * pdf_diffuse + (1.f - diffuse_prob) * pdf_specular;

        break;
    }
    }
}

Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N)
{
    switch (m_type)
    {
    case DIFFUSE:
    {
        // calculate the contribution of diffuse   model
        float cosalpha = dotProduct(N, wo);
        if (cosalpha > 0.0f)
        {
            Vector3f diffuse = albedo / M_PI;
            return diffuse;
        }
        else
            return Vector3f(0.0f);
        break;
    }
    case MICROFACET:
    {
        float N_dot_wi = std::max(0.f, dotProduct(N, wi));
        float N_dot_wo = std::max(0.f, dotProduct(N, wo));

        if (N_dot_wo > 0.f && N_dot_wi > 0.f)
        {
            Vector3f h = (wi + wo).normalized();
            float h_dot_wi = std::max(0.f, dotProduct(h, wi));
            float N_dot_h = std::max(0.f, dotProduct(N, h));

            Vector3f F_term = fresnelSchlick(h_dot_wi, F_0);
            float G_term = geometrySchlickSmith(N_dot_wi) * geometrySchlickSmith(N_dot_wo);
            float D_term = distributionGGX(N_dot_h);

            Vector3f specular = F_term * G_term * D_term /
                                std::max(EPSILON, 4.f * N_dot_wi * N_dot_wo);

            Vector3f Kd = (Vector3f(1.f) - F_term) * (1.f - metallic);
            Vector3f diffuse = Kd * albedo / M_PI;
            return specular + diffuse;
            // return specular;
        }
        else
        {
            return Vector3f(0.f);
        }
        break;
    }
    }
}

Vector3f Material::fresnelSchlick(const float h_dot_wi, const Vector3f &F_0)
{
    float complement = 1.f - h_dot_wi;
    return F_0 + (Vector3f(1.f) - F_0) * complement * complement *
                     complement * complement * complement;
}

float Material::geometrySchlickSmith(const float N_dot_w)
{
    float complement = 1.f - k;
    return N_dot_w / (complement * N_dot_w + k);
}

float Material::distributionGGX(const float N_dot_h)
{

    float N_dot_h_square = N_dot_h * N_dot_h;
    float denom = N_dot_h_square * (alpha - 1.f) + 1.f;
    denom = std::max(M_PI * denom * denom, EPSILON);
    return alpha / denom;
}
#endif // RAYTRACING_MATERIAL_H
