module.exports = function (context, options) {
  return {
    name: "docusaurus-plugin-webpack-config",
    configureWebpack(config, isServer) {
      return {
        // Disable persistent cache on Vercel to avoid serialization issues
        cache: process.env.VERCEL ? false : config.cache,
        externals: isServer
          ? {
              bufferutil: "bufferutil",
              "utf-8-validate": "utf-8-validate",
              ws: "ws",
            }
          : {},
        resolve: {
          fallback: isServer
            ? {}
            : {
                fs: false,
                net: false,
                tls: false,
                crypto: false,
                path: false,
                stream: false,
                util: false,
                http: false,
                https: false,
                zlib: false,
                querystring: false,
                url: false,
                os: false,
              },
        },
      };
    },
  };
};
