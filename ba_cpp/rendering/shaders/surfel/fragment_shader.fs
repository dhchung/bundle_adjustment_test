#version 330 core
out vec4 FragColor;

uniform vec3 objectNormal;
uniform vec3 lightDir;
uniform vec3 lightColor;
uniform vec3 objectColor;

void main()
{
        //Ambient
        float ambientStrength = 0.2;
        vec3 ambient = ambientStrength * lightColor;

        vec3 norm = normalize(objectNormal);
        vec3 lightDir_n = normalize(lightDir);

        float diff = max(dot(norm, lightDir_n), 0.0);
        vec3 diffuse = diff*lightColor;

        vec3 result = (ambient + diffuse)*objectColor;
        FragColor = vec4(result, 1.0f);

}