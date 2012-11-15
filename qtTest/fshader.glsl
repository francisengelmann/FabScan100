#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform sampler2D texture;

varying vec3 v_col;

//! [0]
void main()
{
    // Set fragment color from texture
    gl_FragColor = vec4(v_col, 0.5);
    //gl_FragColor = vec4(0.0,1.0,0.0,1.0);
    //gl_FragColor = texture2D(texture, v_texcoord);
}
//! [0]

